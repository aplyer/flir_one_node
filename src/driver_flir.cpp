#include <boost/format.hpp>
#include <opencv2/highgui.hpp>
//#include <driver_base/SensorLevels.h>
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
    camera_frame_("flir"),
    isOk(true),
    states(INIT),
    setup_states(SETUP_INIT),
    context(NULL),
    vendor_id(0x09cb),
    product_id(0x1996),
    it_(new image_transport::ImageTransport(camera_nh_)){
    image_pub_ = priv_nh.advertise<sensor_msgs::Image>("ir_16b/image_raw", 1);
    image_rgb_pub_ = priv_nh.advertise<sensor_msgs::Image>("rgb/image_raw", 1);
    image_8b_pub_ = priv_nh.advertise<sensor_msgs::Image>("ir_8b/image_raw", 1);
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


  void DriverFlir::print_bulk_result(char ep[],char EP_error[], int r, int actual_length, unsigned char buf[]) {
          time_t now1;
          int i;

          now1 = time(NULL);
          if (r < 0) {
                 if (strcmp (EP_error, libusb_error_name(r))!=0) {
                     strcpy(EP_error, libusb_error_name(r));
                     fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s: %s\n", ctime(&now1), ep , libusb_error_name(r));
                     sleep(1);
                 }
                 //return 1;
         } else {
             ROS_INFO("\n: %s bulk read EP %s, actual length %d\nHEX:\n",ctime(&now1), ep ,actual_length);
             // write frame to file
   /*
             char filename[100];
             sprintf(filename, "EP%s#%05i.bin",ep,filecount);
             filecount++;
             FILE *file = fopen(filename, "wb");
             fwrite(buf, 1, actual_length, file);
             fclose(file);
   */
           // hex print of first byte
             /*for (i = 0; i <  (((200)<(actual_length))?(200):(actual_length)); i++) {
                     ROS_INFO(" %02x", buf[i]);
             }

             ROS_INFO("\nSTRING:\n");
             for (i = 0; i <  (((200)<(actual_length))?(200):(actual_length)); i++) {
                     if(buf[i]>31) {ROS_INFO("%c", buf[i]);}
             }
             ROS_INFO("\n");*/

         }
  }

  void DriverFlir::read(char ep[],char EP_error[], int r, int actual_length, unsigned char buf[]) {
    // reset buffer if the new chunk begins with magic bytes or the buffer size limit is exceeded
    unsigned char magicbyte[4]={0xEF,0xBE,0x00,0x00};

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
      ROS_ERROR("Reset buffer because of bad Magic Byte!");
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
      ROS_ERROR("wait for next chunk");
      return;
    }
	ros::Time stamp = ros::Time::now();
    int i,v;
    // get a full frame, first print status
    t1=t2;
    gettimeofday(&t2, NULL);
    // fps as moving average over last 20 frames
    fps_t = (19*fps_t+10000000/(((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec)))/20;

    ROS_INFO("#%lld/10 fps:",fps_t);
    ROS_INFO("FrameSize %d ",FrameSize);
    ROS_INFO("ThermalSize %d ",ThermalSize);
    ROS_INFO("JpgSize %d ",JpgSize);
    ROS_INFO("StatusSize %d ",StatusSize);

    /*for (i = 0; i <  StatusSize; i++) {
      v=28+ThermalSize+JpgSize+i;
      if(buf85[v]>31) {ROS_INFO("%c", buf85[v]);}
    }
    ROS_INFO("\n");*/

    unsigned short pix[160*120];
    for (uint8_t y = 0; y < 120; ++y) {
      for (uint8_t x = 0; x < 160; ++x) {
        if (x<80) {
          v = buf85[2*(y * 164 + x) +32]+256*buf85[2*(y * 164 + x) +33];
        }else {
          v = buf85[2*(y * 164 + x) +32+4]+256*buf85[2*(y * 164 + x) +33+4];
        }
        pix[y * 160 + x] = v;   // unsigned char!!
      }
    }

    cv_bridge::CvImage out_msg;
	cv::Mat im16 = cv::Mat (120, 160, CV_16UC1, pix);
    out_msg.header.frame_id = camera_frame_;
	out_msg.header.stamp = stamp;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
    out_msg.image    = 	im16;

    image_pub_.publish(out_msg.toImageMsg());


	cv::Mat rawRgb = cv::Mat(1, JpgSize, CV_8UC1, &buf85[28+ThermalSize]);
	cv::Mat decodedImage  =  cv::imdecode( rawRgb, CV_LOAD_IMAGE_COLOR);
	cv_bridge::CvImage out_rgb;
	out_rgb.header.frame_id = camera_frame_;
	out_rgb.header.stamp = stamp;
    out_rgb.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
	out_rgb.image   =  decodedImage;
    image_rgb_pub_.publish(out_rgb.toImageMsg());


// Max & Min value used for scaling
// (limits: -20° - +75° | 1600 - 5852)
// 
// Theorical IR Sensor sensitivity : 0.1°C
	int max = 3847; // <=>  40°C
	int min = 2934; // <=>  20°C
	int delta = max - min;
	cv::Mat im8b = 255*(im16-min)/(max-min);
    im8b.convertTo(im8b , CV_8UC1);
	cv_bridge::CvImage out_8b;
    out_8b.header.frame_id = camera_frame_;
	out_8b.header.stamp = stamp;
    out_8b.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
    out_8b.image    =  im8b;
	image_8b_pub_.publish(out_8b.toImageMsg());



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
        // endless loop
        // poll Frame Endpoints 0x85
        // don't change timeout=100ms !!
        r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length, 200);
        switch(r){
          case LIBUSB_ERROR_TIMEOUT:
            ROS_ERROR("LIBUSB_ERROR_TIMEOUT");
            break;
          case LIBUSB_ERROR_PIPE:
            ROS_ERROR("LIBUSB_ERROR_PIPE");
            break;
          case LIBUSB_ERROR_OVERFLOW:
            ROS_ERROR("LIBUSB_ERROR_OVERFLOW");
            break;
          case LIBUSB_ERROR_NO_DEVICE:
            ROS_ERROR("LIBUSB_ERROR_NO_DEVICE");
            break;
        }
        if (actual_length > 0){
          ROS_INFO("T'es une FRAME %d", actual_length );
          read("0x85",EP85_error, r, actual_length, buf);
        }
      }
        break;

      case ERROR:
        isOk = false;
        break;
    }

    // poll Endpoints 0x81, 0x83
    r = libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);
    print_bulk_result("0x81",EP81_error, r, actual_length, buf);

    r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
    print_bulk_result("0x83",EP83_error, r, actual_length, buf);
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
