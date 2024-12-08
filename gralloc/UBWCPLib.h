#ifndef UBWCPLIB_H
#define UBWCPLIB_H

typedef enum {
    UBWCPLib_RGBA8888 = 0,
    UBWCPLib_NV12,
    UBWCPLib_TP10,
    UBWCPLib_P010,
    UBWCPLib_RGB,
    UBWCPLib_YUV,
    UBWCPLib_UNKNOWN,
    UBWCPLib_NUM_FORMATS
} UBWCPLib_Image_Format;

typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int stride;
    UBWCPLib_Image_Format image_format;
    unsigned int scanlines;
    unsigned int planar_padding;
} UBWCPLib_buf_attrs;

int LINK_UBWCPLib_get_stride_alignment(void *context, UBWCPLib_Image_Format format, size_t *stride);
int LINK_UBWCPLib_validate_stride(void *context, unsigned int width, UBWCPLib_Image_Format format, unsigned int height);
int LINK_UBWCPLib_set_buf_attrs(void *context, unsigned int width, UBWCPLib_buf_attrs *attrs);
void* LINK_UBWCPLib_create_session();
void LINK_UBWCPLib_destroy_session(void *session);

#endif // UBWCPLIB_H
