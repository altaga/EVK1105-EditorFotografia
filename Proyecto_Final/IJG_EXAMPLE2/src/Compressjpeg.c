// #include <stdio.h>
// 
// /*
//  * Include file for users of JPEG library.
//  * You will need to have included system headers that define at least
//  * the typedefs FILE and size_t before you can include jpeglib.h.
//  * (stdio.h is sufficient on ANSI-conforming systems.)
//  * You may also wish to include "jerror.h".
//  */
// 
// #include "jpeglib.h"
// 
// /*
//  * <setjmp.h> is used for the optional error recovery mechanism shown in
//  * the second part of the example.
//  */
// 
// #include <setjmp.h>
// 
// 
// 
// /******************** JPEG COMPRESSION SAMPLE INTERFACE *******************/
// 
// /* This half of the example shows how to feed data into the JPEG compressor.
//  * We present a minimal version that does not worry about refinements such
//  * as error recovery (the JPEG code will just exit() if it gets an error).
//  */
// 
// 
// /*
//  * IMAGE DATA FORMATS:
//  *
//  * The standard input image format is a rectangular array of pixels, with
//  * each pixel having the same number of "component" values (color channels).
//  * Each pixel row is an array of JSAMPLEs (which typically are unsigned chars).
//  * If you are working with color data, then the color values for each pixel
//  * must be adjacent in the row; for example, R,G,B,R,G,B,R,G,B,... for 24-bit
//  * RGB color.
//  *
//  * For this example, we'll assume that this data structure matches the way
//  * our application has stored the image in memory, so we can just pass a
//  * pointer to our image buffer.  In particular, let's say that the image is
//  * RGB color and is described by:
//  */
// 
// extern JSAMPLE * image_buffer;	/* Points to large array of R,G,B-order data */
// extern int image_height;	/* Number of rows in image */
// extern int image_width;		/* Number of columns in image */
// 
// 
// /*
//  * Sample routine for JPEG compression.  We assume that the target file name
//  * and a compression quality factor are passed in.
//  */
// 
// GLOBAL(void)
// void *jpeg_lib_decode_ex(int offset, U16 *width, U16 *height)
// {
//   struct jpeg_compress_struct *cinfo = (struct jpeg_compress_struct *) jpeg_lib_data.cinfo;
//   struct extended_error_mgr *jerr = (struct extended_error_mgr *) cinfo->err;
//   cinfo.image_width = image_width; 	/* image width and height, in pixels */
//   cinfo.image_height = image_height;
//   uint16_t max_width, max_height;
//   uint16_t max_lines = 1;
//   uint16_t scale_denom;
// // set output image position for the JPEG library
//   jpeg_out_buffer_pos = (uint16_t *) jpeg_lib_data.output_image;
//   stream_open();				
//   if(offset)
//   stream_seek(offset);	
//   jpeg_stdio_src(cinfo, 0);
//   int row_stride;		/* physical row width in image buffer */
//   cinfo.err = jpeg_std_error(&jerr);
//   jpeg_create_compress(&cinfo);
//   cinfo.input_components = 3;		/* # of color components per pixel */
//   cinfo.in_color_space = JCS_RGB565; 	/* colorspace of input image */
//   jpeg_set_defaults(&cinfo);
//   jpeg_set_quality(&cinfo, 100, TRUE /* limit to baseline-JPEG values */);
//   jpeg_start_compress(&cinfo, TRUE);
//   row_stride = image_width * 3;	/* JSAMPLEs per row in image_buffer */
//  while (cinfo.next_scanline < cinfo.image_height)
//   	{
// 	  	jpeg_write_scanlines(cinfo, NULL, max_lines);
//   	}
//   jpeg_finish_compress(&cinfo);
//   stream_close();	
//   jpeg_destroy_compress(&cinfo);
// }