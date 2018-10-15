typedef struct
{
   double x; 
   double y; 
   double theta;
   double xv;
   double yv;
   double thetav;
   unsigned long long timestamp_sec;
   unsigned long long timestamp_nsec;
   char* host;
}OdoData;

typedef struct
{
   int id;
  int  laser_type;  /**< what kind of laser is this SICK is 4**/
  double start_angle;                     /**< angle of the first beam relative -DEG2RAD(ConfigData->fFov * 0.5)**/
                                          /**< to to the center of the laser **/
  double fov;                             /**< field of view of the laser  DEG2RAD(270) for omniRob**/
  double angular_resolution;              /**< angular resolution of the laser DEG2RAD(0.5)**/
  double maximum_range;                   /**< the maximum valid range of a measurement 29. **/
  double accuracy;                        /**< error in the range measurements 0.01 **/
  int remission_mode;                     /**< if and what kind of remission values are used  (0 = no remission available used for omniRob*/
   int num_readings;                     /**< Number of beams in this message 541 for omniRob **/
   float* range;                         /**< Array of proximity measurements **/
   int num_remissions;                   /**< Number of remission values (0 = no remission available **/
   float* remission;                    /**< Array of remission measurements  **/
   double laser_pose_x;
   double laser_pose_y;
   double laser_pose_theta;           			   /**< Position of the center of the laser **/
   double robot_pose_x;
   double robot_pose_y;
   double robot_pose_theta;                        /**< Position of the center of the robot **/
   double tv;
   double rv;                      /**< Translational and rotational velocities **/
   unsigned long long  timestamp_sec;            /**< Timestamp when the laser data was recorded (received by the devide driver) **/
   unsigned long long  timestamp_nsec;            /**< Timestamp when the laser data was recorded (received by the devide driver) **/
   char* host;                    /**< The host from which this message was sent **/
}LaserMsg;
