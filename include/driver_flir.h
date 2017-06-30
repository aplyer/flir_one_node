#include <boost/thread/mutex.hpp>

#include <libusb.h>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/fill_image.h>

/** @file

    @brief ROS driver interface for UEYE-compatible USB digital cameras.

*/
#define BUF85SIZE 1048576

namespace driver_flir
{

  class DriverFlir
  {
  public:

    DriverFlir( ros::NodeHandle nh,
                ros::NodeHandle priv_nh,
                ros::NodeHandle camera_nh);
    ~DriverFlir();
    void poll(void);
    void setup(void);
    void shutdown(void);

    bool ok();

  private:
    void publish(const sensor_msgs::ImagePtr &image);
    void read(char ep[],char EP_error[], int r, int actual_length, unsigned char buf[]);

    void print_bulk_result(char ep[],char EP_error[], int r, int actual_length, unsigned char buf[]);

    libusb_context *context;
    struct libusb_device_handle *devh;

    unsigned char buf[1048576];
    int actual_length;
   	char EP81_error[50];
    char EP83_error[50];
    char EP85_error[50];
    int buf85pointer = 0;
    unsigned char buf85[BUF85SIZE];

    enum states_t {INIT, INIT_1, INIT_2, ASK_ZIP, ASK_VIDEO, POOL_FRAME, ERROR};
    states_t states;

    enum setup_states_t {SETUP_INIT, SETUP_LISTING, SETUP_FIND, SETUP_SET_CONF, SETUP_CLAIM_INTERFACE_0,  SETUP_CLAIM_INTERFACE_1,  SETUP_CLAIM_INTERFACE_2, SETUP_ALL_OK, SETUP_ERROR};
    setup_states_t setup_states;

    int error_code;

    bool isOk;

    long long fps_t;
    struct timeval t1, t2;

    int vendor_id;
    int product_id;


    ros::NodeHandle nh_;                  // node handle
    ros::NodeHandle priv_nh_;             // private node handle
    ros::NodeHandle camera_nh_;           // camera name space handle
    std::string camera_name_;             // camera name
    std::string camera_frame_;             // camera name

    /** image transport interfaces */
    boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::Publisher image_pub_;
    ros::Publisher image_rgb_pub_;
    ros::Publisher image_8b_pub_;
  };
};
