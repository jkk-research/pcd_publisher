/*
 *  pcd_lib.h (v0.0.1)
 *  Point Cloud Data library for ROS 2 written by Adam Hunyadvari
 *
 *  Adam Hunyadvari <hunyadvari.adam.rado@hallgato.sze.hu>
 */
#ifndef PCD_LIB_H
#define PCD_LIB_H

#define PCD_LIB static
#define PCD_VERSION ".7"
#define PCD_VERSION_STR "VERSION "PCD_VERSION"\n"

#define PCD_DEFAULT_VIEWPOINT_STR "VIEWPOINT 0 0 0 1 0 0 0\n"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <stdlib.h>
#include <fcntl.h> // for open, close, write syscalls
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#define PCD_LIB_ALLOC(x) malloc(x)
#define PCD_LIB_FREE(x)  free(x)

#define KB(x) ((x) * 1024)
#define MB(x) (KB(x) * 1024)

typedef enum {
	PCD_TYPE_ASCII,
	PCD_TYPE_BINARY,
} Pcd_Type;

inline void pcd_append_header(char *header_buffer, size_t *header_size, const char *rhs, size_t rhs_size) {
	memcpy(header_buffer + *header_size, rhs, rhs_size);
	*header_size += rhs_size;
}

PCD_LIB size_t pcd_get_size_from_datatype(uint8_t datatype) {
	static const size_t datatype_sizes[] = {
		0, // 0 (default value)
		1, // 1
		1, // 2
		2, // 3
		2, // 4
		4, // 5
		4, // 6
		4, // 7
		8, // 8
	};
	return (datatype < sizeof(datatype_sizes) / sizeof(datatype_sizes[0]))
		? datatype_sizes[datatype]
		: 0;
}

PCD_LIB char pcd_get_type_from_datatype(uint8_t datatype) {
	static const char datatype_types[] = {
		'\0', // (default value)
		'I', // 1
		'U', // 2
		'I', // 3
		'U', // 4
		'I', // 5
		'U', // 6
		'F', // 7
		'F', // 8
	};
	return (datatype < sizeof(datatype_types) / sizeof(datatype_types[0]))
		? datatype_types[datatype]
		: '\0';
}

#define UINT_CONVERT_TEMPLATE(buffer_size, value, buffer) \
	char temp[buffer_size]; \
	int i = 0; \
	if (value == 0) { \
		buffer[0] = '0'; \
		return 1; \
	} \
	while (value > 0) { \
		temp[i++] = (value % 10) + '0'; \
		value /= 10; \
	} \
	int j = 0; \
	while (i > 0) { \
		buffer[j++] = temp[--i]; \
	} \
	return j; \

PCD_LIB size_t pcd_uint8_to_string(uint8_t value, char *buffer) {
	UINT_CONVERT_TEMPLATE(3, value, buffer)
}

PCD_LIB size_t pcd_uint16_to_string(uint16_t value, char *buffer) {
	UINT_CONVERT_TEMPLATE(5, value, buffer)
}

// Convert the uint32_t to string then returns the number of characters
// IMPORTANT: this returns a non null terminated string
PCD_LIB size_t pcd_uint32_to_string(uint32_t value, char *buffer) {
	UINT_CONVERT_TEMPLATE(10, value, buffer)
}

PCD_LIB size_t pcd_uint64_to_string(uint64_t value, char *buffer) {
	UINT_CONVERT_TEMPLATE(20, value, buffer)
}

#define INT_CONVERT_TEMPLATE(buffer_size, value, buffer) \
	char temp[buffer_size]; \
	int i = 0; \
	int sign = value; \
	if (value < 0) \
		value = -value; \
	if (value == 0) { \
		buffer[0] = '0'; \
		return 1; \
	} \
	while (value > 0) { \
		temp[i++] = (value % 10) + '0'; \
		value /= 10; \
	} \
	int j = 0; \
	if (sign < 0) { \
		buffer[j++] = '-'; \
	} \
	while (i > 0) { \
		buffer[j++] = temp[--i]; \
	} \
	return j; \

PCD_LIB size_t pcd_int8_to_string(int8_t value, char *buffer) {
    if (value == -128) {
        memcpy(buffer, "-128", 4);
        return 4;
    }
	INT_CONVERT_TEMPLATE(4, value, buffer)
}

PCD_LIB size_t pcd_int16_to_string(int16_t value, char *buffer) {
    if (value == -32768) {
        memcpy(buffer, "-32768", 6);
        return 6;
    }
	INT_CONVERT_TEMPLATE(6, value, buffer)
}

PCD_LIB size_t pcd_int32_to_string(int32_t value, char *buffer) {
    if (value == -2147483648) {
        memcpy(buffer, "-2147483648", 11);
        return 11;
    }
	INT_CONVERT_TEMPLATE(11, value, buffer)
}

// NOTE: you can optimize this by writing your own conversion function
PCD_LIB size_t pcd_float32_to_string(float value, char *buffer) {
	snprintf(buffer, 64, "%e", value);
	return strlen(buffer);
}

PCD_LIB size_t pcd_float64_to_string(double value, char *buffer) {
	snprintf(buffer, 64, "%e", value);
	return strlen(buffer);
}

#define HEADER_MAX_SIZE KB(8)
PCD_LIB bool pcd_write_header(rclcpp::Logger logger, const sensor_msgs::msg::PointCloud2::SharedPtr msg, int fd, Pcd_Type type) {
	char *header_buffer = (char *)PCD_LIB_ALLOC(HEADER_MAX_SIZE);
	if (header_buffer == NULL) {
		RCLCPP_ERROR(logger, "Couldn't allocate memory for pcd header");
		return false;
	}

	size_t header_size  = 0;


	pcd_append_header(header_buffer, &header_size, PCD_VERSION_STR, sizeof(PCD_VERSION_STR) - 1);

	pcd_append_header(header_buffer, &header_size, "FIELDS", sizeof("FIELDS") - 1);
	for (const auto &field: msg->fields) {
		pcd_append_header(header_buffer, &header_size, " ", 1);
		pcd_append_header(header_buffer, &header_size, field.name.c_str(), field.name.length());
	}
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	// TODO: optimalization by merge SIZE and TYPE (and maybe COUNT?)
	pcd_append_header(header_buffer, &header_size, "SIZE", sizeof("SIZE") - 1);
	for (const auto &field: msg->fields) {
		char size_str = '0' + pcd_get_size_from_datatype(field.datatype);
		pcd_append_header(header_buffer, &header_size, " ", 1);
		pcd_append_header(header_buffer, &header_size, &size_str, 1);
	}
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	pcd_append_header(header_buffer, &header_size, "TYPE", sizeof("TYPE") - 1);
	for (const auto &field: msg->fields) {
		char type_str = pcd_get_type_from_datatype(field.datatype);
		pcd_append_header(header_buffer, &header_size, " ", 1);
		pcd_append_header(header_buffer, &header_size, &type_str, 1);
	}
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	pcd_append_header(header_buffer, &header_size, "COUNT", sizeof("COUNT") - 1);
	for (const auto &field: msg->fields) {
		char count_str[10]; // Max(uint32 type) = 4,294,967,295 which contains 10 digits
		size_t count_len = pcd_uint32_to_string(field.count, count_str);
		pcd_append_header(header_buffer, &header_size, " ", 1);
		pcd_append_header(header_buffer, &header_size, (const char*)&count_str, count_len);
	}
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	char tmp_buffer[20];
	size_t n;
	pcd_append_header(header_buffer, &header_size, "WIDTH ", sizeof("WIDTH ") - 1);
	n = pcd_uint32_to_string(msg->width, tmp_buffer);
	pcd_append_header(header_buffer, &header_size, (const char*)&tmp_buffer, n);
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	pcd_append_header(header_buffer, &header_size, "HEIGHT ", sizeof("HEIGHT ") - 1);
	n = pcd_uint32_to_string(msg->height, tmp_buffer);
	pcd_append_header(header_buffer, &header_size, (const char*)&tmp_buffer, n);
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	pcd_append_header(header_buffer, &header_size, PCD_DEFAULT_VIEWPOINT_STR, sizeof(PCD_DEFAULT_VIEWPOINT_STR) - 1);

	uint64_t num_of_points = (uint64_t)msg->width * (uint64_t)msg->height;
	pcd_append_header(header_buffer, &header_size, "POINTS ", sizeof("POINTS ") - 1);
	n = pcd_uint64_to_string(num_of_points, tmp_buffer);
	pcd_append_header(header_buffer, &header_size, (const char*)&tmp_buffer, n);
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	const char *file_type = (type == PCD_TYPE_ASCII) ? "ascii" : "binary";
	pcd_append_header(header_buffer, &header_size, "DATA ", sizeof("DATA ") - 1);
	pcd_append_header(header_buffer, &header_size, file_type, strlen(file_type));
	pcd_append_header(header_buffer, &header_size, "\n", 1);

	// Write all at once for speed
	if (write(fd, header_buffer, header_size) == -1) {
		PCD_LIB_FREE(header_buffer);
		return false;
	}

	PCD_LIB_FREE(header_buffer);
	return true;
} 

typedef struct Pcd_Ascii_Data_Buffer {
	char *buffer;
	size_t len;
} Pcd_Ascii_Data_Buffer;

#define PCD_DEFAULT_ASCII_DATA_BUFFER MB(64)
PCD_LIB bool pcd_append_write_to_ascii_file(Pcd_Ascii_Data_Buffer *data_buffer, int fd, const char *rhs, size_t rhs_size) {
	if (data_buffer->len + rhs_size > PCD_DEFAULT_ASCII_DATA_BUFFER) {
		if (write(fd, data_buffer->buffer, data_buffer->len) == -1) {
			return false;
		}
		data_buffer->len = 0;
	}
	memcpy(data_buffer->buffer + data_buffer->len, rhs, rhs_size);
	data_buffer->len += rhs_size;
	return true;
}

PCD_LIB bool pcd_write_buffer_to_ascii_file(Pcd_Ascii_Data_Buffer *data_buffer, int fd) {
	if (data_buffer->len == 0)
		return true;

	return write(fd, data_buffer->buffer, data_buffer->len) != -1;
}

PCD_LIB bool pcd_save_to_ascii_file(rclcpp::Logger logger, const sensor_msgs::msg::PointCloud2::SharedPtr msg, char *filename) {
	auto header = msg->header;
	auto timestamp = header.stamp;
	RCLCPP_INFO(logger, "Frame id: %s", header.frame_id.c_str());
	RCLCPP_INFO(logger, "Timestamp: sec %d; nanosec: %d", timestamp.sec, timestamp.nanosec);
	RCLCPP_INFO(logger, "Width: %d Height: %d", msg->width, msg->height);

	// Open the file descriptor
	int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd == -1) {
		RCLCPP_ERROR(logger, "Failed to open '%s' file for writing: %s", filename, strerror(errno));
		return false;
	}

	// Write the header first
	if (!pcd_write_header(logger, msg, fd, PCD_TYPE_ASCII)) {
		RCLCPP_ERROR(logger, "Error writing to file '%s': %s", filename, strerror(errno));
		close(fd);
		return false;
	}

	Pcd_Ascii_Data_Buffer data_buffer;
	data_buffer.buffer = (char *)PCD_LIB_ALLOC(PCD_DEFAULT_ASCII_DATA_BUFFER);
	data_buffer.len = 0;

	if (data_buffer.buffer == NULL) {
		RCLCPP_ERROR(logger, "Failed to allocate memory for data_buffer");
		close(fd);
		return false;
	}

	uint64_t num_of_points = (uint64_t)msg->width * (uint64_t)msg->height;
	for (uint64_t point_idx = 0; point_idx < num_of_points; ++point_idx) {
		uint64_t point_offset = (uint64_t)msg->point_step * point_idx;
		bool need_space = false;
		for (const auto &field: msg->fields) {
			uint8_t *data_ptr = msg->data.data() + point_offset + field.offset;
			size_t elem_size = pcd_get_size_from_datatype(field.datatype);
			for (uint32_t count_idx = 0; count_idx < field.count; ++count_idx) {
				uint8_t *final_ptr = data_ptr + count_idx * elem_size;

				char number_buffer[64];
				size_t n;
				switch (field.datatype) {
					case 1: // INT8
						n = pcd_int8_to_string(*((int8_t *)final_ptr), number_buffer);
						break;
					case 2: // UINT8
						n = pcd_uint8_to_string(*((uint8_t *)final_ptr), number_buffer);
						break;
					case 3: // INT16
						n = pcd_int16_to_string(*((int16_t *)final_ptr), number_buffer);
						break;
					case 4: // UINT16
						n = pcd_uint16_to_string(*((uint16_t *)final_ptr), number_buffer);
						break;
					case 5: // INT32
						n = pcd_int32_to_string(*((int32_t *)final_ptr), number_buffer);
						break;
					case 6: // UINT32
						n = pcd_uint32_to_string(*((uint32_t *)final_ptr), number_buffer);
						break;
					case 7: // FLOAT32
						n = pcd_float32_to_string(*((float *)final_ptr), number_buffer);
						break;
					case 8: // FLOAT64
						n = pcd_float64_to_string(*((double *)final_ptr), number_buffer);
						break;
				}

				if (need_space) {
					pcd_append_write_to_ascii_file(&data_buffer, fd, " ", 1);
				} else {
					need_space = true;
				}

				pcd_append_write_to_ascii_file(&data_buffer, fd, number_buffer, n);
			} 
		}
		pcd_append_write_to_ascii_file(&data_buffer, fd, "\n", 1);
	}

	if (!pcd_write_buffer_to_ascii_file(&data_buffer, fd)) {
		RCLCPP_ERROR(logger, "Failed to write to ascii PCD file");
		close(fd);
		return false;
	}

    RCLCPP_INFO(logger, "---------------%s------------", filename);
    for (const auto &field: msg->fields) {
        RCLCPP_INFO(logger, "Field name: %s", field.name.c_str());
        RCLCPP_INFO(logger, "Field offset: %d", field.offset);
        RCLCPP_INFO(logger, "Field datatype: %d", field.datatype);
        RCLCPP_INFO(logger, "Field count: %d", field.count);
    }
    RCLCPP_INFO(logger, "----------------------------------");


    // Close the file descriptor
    close(fd);
    return true;
}

#endif /* PCD_LIB_H */
