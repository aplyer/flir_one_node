#include <boost/format.hpp>

#include <driver_base/SensorLevels.h>
#include "driver_flir.h"


namespace driver_flir
{

  DriverFlir::DriverFlir( ros::NodeHandle nh,
                        ros::NodeHandle priv_nh,
                        ros::NodeHandle camera_nh):
    nh_(nh),
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    camera_name_("FLIR_USB"),
    isOk(true),
    states(INIT),
    setup_states(SETUP_INIT),
    context(NULL),
    vendor_id(0x09cb),
    product_id(0x1996),
    it_(new image_transport::ImageTransport(camera_nh_)){
    image_pub_ = priv_nh.advertise<sensor_msgs::Image>("image_raw", 1);
  }

  DriverFlir::~DriverFlir() {
  }

  void DriverFlir::shutdown() {
    libusb_reset_device(devh);
    libusb_close(devh);
    libusb_exit(NULL);
    isOk = false;
  }

  bool DriverFlir::ok(void){
    return isOk;
  }

  void DriverFlir::publish(const sensor_msgs::ImagePtr &image) {
    image_pub_.publish(image);
  }

  void DriverFlir::read(char ep[],char EP_error[], int r, int actual_length, unsigned char buf[]) {
    // reset buffer if the new chunk begins with magic bytes or the buffer size limit is exceeded
    unsigned char magicbyte[4]={0xEF,0xBE,0x00,0x00};
    #define BUF85SIZE 1048576
    int buf85pointer = 0;
    unsigned char buf85[BUF85SIZE];

    if  ((strncmp (( const char *)buf, ( const char *)magicbyte,4)==0 ) || ((buf85pointer + actual_length) >= BUF85SIZE)) {
      //printf(">>>>>>>>>>>reset buff pointer<<<<<<<<<<<<<\n");
      buf85pointer=0;
    }

    //printf("actual_length %d !!!!!\n", actual_length);

    memmove(buf85+buf85pointer, buf, actual_length);
    buf85pointer=buf85pointer+actual_length;

    if  ((strncmp (( const char *)buf85, ( const char *)magicbyte,4)!=0 )) {
      //reset buff pointer
      buf85pointer=0;
      //printf("Reset buffer because of bad Magic Byte!\n");
      return;
    }

    // a quick and dirty job for gcc
    uint32_t FrameSize   = buf85[ 8] + (buf85[ 9] << 8) + (buf85[10] << 16) + (buf85[11] << 24);
    uint32_t ThermalSize = buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
    uint32_t JpgSize     = buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);
    uint32_t StatusSize  = buf85[20] + (buf85[21] << 8) + (buf85[22] << 16) + (buf85[23] << 24);

    //printf("FrameSize= %d (+28=%d), ThermalSize %d, JPG %d, StatusSize %d, Pointer %d\n",FrameSize,FrameSize+28, ThermalSize, JpgSize,StatusSize,buf85pointer);

    if ( (FrameSize+28) > (buf85pointer) ) {
      // wait for next chunk
      return;
    }

    int i,v;
    // get a full frame, first print status
    t1=t2;
    gettimeofday(&t2, NULL);
    // fps as moving average over last 20 frames
    fps_t = (19*fps_t+10000000/(((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec)))/20;

    ROS_INFO("#%lld/10 fps:",fps_t);
    for (i = 0; i <  StatusSize; i++) {
      v=28+ThermalSize+JpgSize+i;
      if(buf85[v]>31) {ROS_INFO("%c", buf85[v]);}
    }
    ROS_INFO("\n");
/*
    buf85pointer=0;

    unsigned short pix[160*120];  // original Flir 16 Bit RAW
    int x, y;
    unsigned char *fb_proc, *fb_proc2, *fb_proc4;
    unsigned short *fb_proc3;

    fb_proc = malloc(160 * 120); // 8 Bit gray buffer
    fb_proc2 = malloc(160 * 120 * 3); // 16-bit gray buffer (include in a 24 bits image)
    fb_proc3 = malloc(160 * 120 * 2); // 16-bit gray buffer
    fb_proc4 = malloc(160 * 120 * 3); // 8x8x8 bits

    for (y = 0; y < 120; ++y) {
      for (x = 0; x < 160; ++x) {
        if (x<80) {
          v = buf85[2*(y * 164 + x) +32]+256*buf85[2*(y * 164 + x) +33];
        }else {
          v = buf85[2*(y * 164 + x) +32+4]+256*buf85[2*(y * 164 + x) +33+4];
        }
        pix[y * 160 + x] = v;   // unsigned char!!
      }
    }


    // Max & Min value used for scaling
    // (limits: -20° - +75° | 1600 - 5852)
    //
    // Theorical IR Sensor sensitivity : 0.1°C

    int max = 3847; // <=>  40°C
    int min = 2934; // <=>  20°C


    int delta = max - min;

    int scale = 0x10000 / delta; // (2¹⁶/delta <=> 65536/delta)

    for (y = 0; y < 120; ++y) {
      for (x = 0; x < 160; ++x) {
        int v = (pix[y * 160 + x] - min) * scale;

        // fb_proc3 is the 16-Bit Gray Image
        fb_proc3[y * 160 + x] = v; //unsigned short

        // High and low 8-bits part of the 16-Bit values
        uint8_t v_low = v & 0xff;
        uint8_t v_high = v >> 8;

        // fb_proc is the 8-bit gray scale frame buffer
        fb_proc[y * 160 + x] = v_high;   // unsigned char!!

        // fb_proc2 is the 16-bit gray scale frame buffer.
        // It's inside a 24 bits image but the with only channels set.
        // The 1st one contains the first 8 bits and the second one the
        // last 8 bits.
        fb_proc2[3*y * 160 + x*3] = 0;
        fb_proc2[(3*y * 160 + x*3) + 1] = v_high;
        fb_proc2[(3*y * 160 + x*3) + 2] = v_low >> 2;


        // fb_proc4 is an 24bit RGB buffer
        const int *colormap = colormap_ironblack;

        fb_proc4[3*y * 160 + x*3] = colormap[3 * v_high];   // unsigned char!!
        fb_proc4[(3*y * 160 + x*3)+1] = colormap[3 * v_high + 1];   // unsigned char!!
        fb_proc4[(3*y * 160 + x*3)+2] = colormap[3 * v_high + 2];   // unsigned char!!
      }
    }*/
  }

  void DriverFlir::poll(void){
 	  unsigned char data[2]={0,0}; // only a bad dummy
    int r = 0;
    time_t now;

    switch (states) {
      /* Flir config
      01 0b 01 00 01 00 00 00 c4 d5
      0 bmRequestType = 01
      1 bRequest = 0b
      2 wValue 0001 type (H) index (L)    stop=0/start=1 (Alternate Setting)
      4 wIndex 01                         interface 1/2
      5 wLength 00
      6 Data 00 00

      libusb_control_transfer (*dev_handle, bmRequestType, bRequest, wValue,  wIndex, *data, wLength, timeout)
      */

      case INIT:
        ROS_INFO("stop interface 2 FRAME\n");
        r = libusb_control_transfer(devh,1,0x0b,0,2,data,0,100);
        if (r < 0) {
          ROS_ERROR("Control Out error %d\n", r);
          error_code = r;
          states = ERROR;
        } else {
          states = INIT_1;
        }
        break;

      case INIT_1:
        ROS_INFO("stop interface 1 FILEIO\n");
        r = libusb_control_transfer(devh,1,0x0b,0,1,data,0,100);
        if (r < 0) {
          ROS_ERROR("Control Out error %d\n", r);
          error_code = r;
          states = ERROR;
        } else {
          states = INIT_2;
        }
        break;

      case INIT_2:
        ROS_INFO("\nstart interface 1 FILEIO\n");
        r = libusb_control_transfer(devh,1,0x0b,1,1,data,0,100);
        if (r < 0) {
          ROS_ERROR("Control Out error %d\n", r);
          error_code = r;
          states = ERROR;
        } else {
          states = ASK_ZIP;
        }
        break;

      case ASK_ZIP:
      {
        ROS_INFO("\nask for CameraFiles.zip on EP 0x83:\n");
        now = time(0); // Get the system time
        ROS_INFO("\n: %s",ctime(&now));

        int transferred = 0;
        char my_string[128];

        //--------- write string: {"type":"openFile","data":{"mode":"r","path":"CameraFiles.zip"}}
        int length = 16;
        unsigned char my_string2[16]={0xcc,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x41,0x00,0x00,0x00,0xF8,0xB3,0xF7,0x00};
        ROS_INFO("\nEP 0x02 to be sent Hexcode: %i Bytes[",length);
        int i;
        for (i = 0; i < length; i++) {
          ROS_INFO(" %02x", my_string2[i]);
        }
        ROS_INFO(" ]\n");

        r = libusb_bulk_transfer(devh, 2, my_string2, length, &transferred, 0);
        if(r == 0 && transferred == length) {
          ROS_INFO("\nWrite successful!");
        }
        else {
          ROS_ERROR("\nError in write! res = %d and transferred = %d\n", r, transferred);
        }

        strcpy(  my_string,"{\"type\":\"openFile\",\"data\":{\"mode\":\"r\",\"path\":\"CameraFiles.zip\"}}");

        length = strlen(my_string)+1;
        ROS_INFO("\nEP 0x02 to be sent: %s", my_string);

        // avoid error: invalid conversion from ‘char*’ to ‘unsigned char*’ [-fpermissive]
        unsigned char *my_string1 = (unsigned char*)my_string;
        //my_string1 = (unsigned char*)my_string;

        r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
        if(r == 0 && transferred == length) {
          ROS_INFO("\nWrite successful!");
          ROS_INFO("\nSent %d bytes with string: %s\n", transferred, my_string);
        }
        else {
          ROS_ERROR("\nError in write! res = %d and transferred = %d\n", r, transferred);
        }

        //--------- write string: {"type":"readFile","data":{"streamIdentifier":10}}
        length = 16;
        unsigned char my_string3[16]={0xcc,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x33,0x00,0x00,0x00,0xef,0xdb,0xc1,0xc1};
        ROS_INFO("\nEP 0x02 to be sent Hexcode: %i Bytes[",length);
        for (i = 0; i < length; i++) {
          ROS_INFO(" %02x", my_string3[i]);
        }
        ROS_INFO(" ]\n");

        r = libusb_bulk_transfer(devh, 2, my_string3, length, &transferred, 0);
        if(r == 0 && transferred == length) {
          ROS_INFO("\nWrite successful!");
        }
        else {
          ROS_ERROR("\nError in write! res = %d and transferred = %d\n", r, transferred);
        }

        //strcpy(  my_string, "{\"type\":\"setOption\",\"data\":{\"option\":\"autoFFC\",\"value\":true}}");
        strcpy(  my_string,"{\"type\":\"readFile\",\"data\":{\"streamIdentifier\":10}}");
        length = strlen(my_string)+1;
        ROS_INFO("\nEP 0x02 to be sent %i Bytes: %s", length, my_string);

        // avoid error: invalid conversion from ‘char*’ to ‘unsigned char*’ [-fpermissive]
        my_string1 = (unsigned char*)my_string;

        r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
        if(r == 0 && transferred == length) {
          ROS_INFO("\nWrite successful!");
          ROS_INFO("\nSent %d bytes with string: %s\n", transferred, my_string);
        }
        else {
          ROS_ERROR("\nError in write! res = %d and transferred = %d\n", r, transferred);
        }

        // go to next state
        now = time(0); // Get the system time
        ROS_INFO("\n: %s",ctime(&now));
        //sleep(1);
        states = ASK_VIDEO;
      }
      break;

      case ASK_VIDEO:
        ROS_INFO("\nAsk for video stream, start EP 0x85:\n");

        r = libusb_control_transfer(devh,1,0x0b,1,2,data, 2,200);
        if (r < 0) {
          ROS_ERROR("Control Out error %d\n", r);
          error_code = r;
          states = ERROR;
        } else {
          states = POOL_FRAME;
        }
      break;

      case POOL_FRAME:
      {
        char EP85_error[50]="";
        // endless loop
        // poll Frame Endpoints 0x85
        // don't change timeout=100ms !!
        r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length, 100);
        if (actual_length > 0){
          read("0x85",EP85_error, r, actual_length, buf);
        }
      }
        break;

      case ERROR:
        isOk = false;
        break;
    }
  }

  void DriverFlir::setup(void){

    do{
      switch (setup_states) {
        case SETUP_INIT:
          if (libusb_init(&context) < 0) {
            ROS_ERROR("failed to initialise libusb");
            setup_states = SETUP_ERROR;
          } else {
            ROS_INFO("Successfully initialise libusb");
            setup_states = SETUP_FIND;
            setup_states = SETUP_LISTING;
          }
          break;

        case SETUP_LISTING:
        {
          int rc = 0;
          libusb_device_handle *dev_handle = NULL   ;
          libusb_device        **devs               ;
          int count = libusb_get_device_list(context, &devs);

         for (size_t idx = 0; idx < count; ++idx) {
            libusb_device *device = devs[idx];
            libusb_device_descriptor desc = {0};

            rc = libusb_get_device_descriptor(device, &desc);
            assert(rc == 0);

            ROS_DEBUG("Vendor:Device = %04x:%04x", desc.idVendor, desc.idProduct);
         }
         libusb_free_device_list(devs, 1); //free the list, unref the devices in it
         setup_states = SETUP_FIND;
        }
          break;

        case SETUP_FIND:
          devh = libusb_open_device_with_vid_pid(context, vendor_id, product_id);
          if ( devh == NULL ) {
            ROS_ERROR_STREAM("Could not find/open device. devh : " << devh);
            setup_states = SETUP_ERROR;
          } else {
            ROS_INFO("Successfully find the Flir One G2 device");
            setup_states = SETUP_SET_CONF;
          }
          break;

        case SETUP_SET_CONF:
          ROS_INFO("A Live");
          if (int r = libusb_set_configuration(devh, 3) < 0) {
            ROS_ERROR("libusb_set_configuration error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            ROS_INFO("Successfully set usb configuration 3");
            setup_states = SETUP_CLAIM_INTERFACE_0;
          }
          break;

        case SETUP_CLAIM_INTERFACE_0:
          if (int r = libusb_claim_interface(devh, 0) <0) {
            ROS_ERROR("libusb_claim_interface 0 error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            ROS_INFO("Successfully claimed interface 1");
            setup_states = SETUP_CLAIM_INTERFACE_1;
          }
          break;

        case SETUP_CLAIM_INTERFACE_1:
          if (int r = libusb_claim_interface(devh, 1) <0) {
            ROS_ERROR("libusb_claim_interface 1 error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            ROS_INFO("Successfully claimed interface 1");
            setup_states = SETUP_CLAIM_INTERFACE_2;
          }
          break;

        case SETUP_CLAIM_INTERFACE_2:
          if (int r = libusb_claim_interface(devh, 2) <0) {
            ROS_ERROR("libusb_claim_interface 2 error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            ROS_INFO("Successfully claimed interface 2");
            setup_states = SETUP_ALL_OK;
          }
          break;

      }
    } while ( (setup_states != SETUP_ERROR) && (setup_states != SETUP_ALL_OK) );

    if (setup_states == SETUP_ERROR){
     	shutdown();
    }

  }

};
