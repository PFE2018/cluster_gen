#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>

#include <boost/shared_ptr.hpp>


#include "adas_msgs/cluster_visualiser.h"


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <math.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <geometry_msgs/Point.h>

//SocketCAN includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <visualization_msgs/Marker.h>

#include <cmath>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <std_msgs/Int8.h>

#include <std_msgs/Float32.h>

#include <std_msgs/builtin_float.h>

#include <time.h>

#include <algorithm>

#include <list>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

#include <tf/transform_broadcaster.h>




#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <ros/ros.h>




/**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **				    			  Classe DisplayWindow	     			     	 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/

class DisplayWindow {

	/***********************************************************************

	 Classe déterminant la position ou se trouve un cluster dans la matrice
	 de position (Danfoss)

	 ***********************************************************************/


	ros::Subscriber sub_actual_speed;


public:

	int window_number(float dist_from_cent, float curv_radius, float heigh);

	void init(ros::NodeHandle& nh);

	void set_max_height(const std_msgs::Float32::ConstPtr& max_height);

};



/**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **				    			  Classe ClusterClass				          	 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/

class ClusterClass {

	/***********************************************************************

	 Classe permettant la définition et la gestion des clusters afin de détecter
	 des obstacles dans une trajectoire linéaire

	 ***********************************************************************/
public:

	float x_;
	float y_;
	float z_;

	float x_moy_;
	float y_moy_;
	float z_moy_;

	int validation_nb_epoq_;

	float nb_point_actu_;

	float cluster_width_;

	float cluster_height_;

	int point_count_;

	int epoq_count_;

	int last_fault_;

	int fault_epoq_count_;

	int fault_;

	int error_type_;

	int id;

//public:

// On ne gère pas la distance en z car la classe cluster n'est appelée que lorsque la distance en z d'un point
// est inférieure à la distance de freinage/facteur de sécurité

	//void init(CLUSTER cluster);

	int is_a_member(float x, float height, float depth);

	int error_type_def(DisplayWindow win, float max_speed);

	// int is_expired(int epoq, int fault);

	ClusterClass(float coor_x, float coor_y, float coor_z,
			float radius_of_curv, float init_steer_angle);

	//void init(float x, float y, float z);

	void dist_update(float actual_speed);

};

/**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **						      Classe TreatmentClass					          	 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/

class TreatmentClass {

	/***********************************************************************

	 Classe qui regroupe les principales caractéristiques de l'ADAS (distance
	 des objets les plus proches, calcule la vitesse maximale du véhicule à
	 l'aide de fonctions prenant en compte la distance de freinage et les
	 informations prélevées sur le point cloud

	 ************************************************************************/

public:
	adas_msgs::cluster_visualiser rviz_marker_msg;

	std::vector<int> cluster_indices;

	float speed_lim_;
	float max_speed_;
	float break_dist_;
	float break_dist_max_speed_;

	float limit_speed_range_;

	int error_type_;     //type pour visualisation penser à comment l'intégrer

	float security_factor_; //Définie la zone de ralentissement par rapport à la zone de freinage

	float actual_speed_;  //Valeur de la vitesse actuelle recue par le noeud CAN

	float distance_from_center_of_rotation_; //Handle for point_cloud points test during turns

	float dist_;   //calculated distance of a point_cloud's point considering a known curved path

	float min_dist_;						//minimum distance memory variable

	float path_width_;

	float closest_clust_distance_;

	int visualisation_index_;

	float vehicule_heigth_;

	float x_;

	float z_;

	float e_brake_always;

	float cluster_zone_index_;

	float cluster_zone_width_index_;

	float cluster_w_;
	float cluster_h_;
	int validation_nb_;

	//Position du point le plus proche
	float z_min_path_;
	float x_min_path_;
	float y_min_path_;

	float distance_between_wheels_;

	float steering_constant_;

	float radius_change_tolerance_;
	float steer_change_tolerance_;

	float steering_angle_;			//Valeur du steering recue par le noeud CAN

	float radius_of_curv_;//Valeur calculer en fonction de l'angle de steering et dimension du véhicule

	int pos_matrix_[25];//Matrice de position pour l'affichage sur ecran Danfoss

	float center_side_path_limit_zone_close;

	float center_side_path_limit_zone_far;

	float max_height_;

	float lateral_secutity_factor_;

    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;

    // Extract the eigenvalues and eigenvectors
    Eigen::Vector3f eigen_value;
    Eigen::Matrix3f eigen_vectors;


    float eigen_1;
    float eigen_2;
    float eigen_3;


    geometry_msgs::Point eigenvals;

    geometry_msgs::Point centroid_point;





    //tf of the kinect

	std_msgs::Int32MultiArray pos_array_;

	DisplayWindow disp_window_;

	visualization_msgs::Marker points_;    	 //Points à visualiser avec Rviz

	std::list<ClusterClass> curv_cluster_;



	 /*********************************************************************************
	 *********************************************************************************/


	ros::Publisher pub_rviz_marker_msg;	//Publie les coordonnés des clusters ainsi que le type d'erreur qui leur est associé


    ros::Publisher pub_centroid;

    ros::Publisher pub_eigenvalues;

	ros::Subscriber sub_kinect_cloud;

	ros::Subscriber sub_actual_speed; //Vitesse actuelle du véhicule telle que reçue par le signal CAN des drives

	ros::Subscriber sub_steering; //Steering angle du véhicule telle que reçue par le signal CAN des drives

	/********************************************************************************

	 Définition de la classe de traitement du point cloud



	 Fonctions:


	 -fonction1
	 Explications :

	 -fonction2
	 Explications :

	 *********************************************************************************/


	void init(ros::NodeHandle& nh);

	void treat(const sensor_msgs::PointCloud2ConstPtr& input);

	void fetch(const std_msgs::Float32::ConstPtr& free_dist);

	void pcl_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input); //test

};


/***********************************************************************

 Classe permettant de définir quel pixels de la matrice de visualisation
 des fautes sur l'écran Danfoss il faut allumer

 ************************************************************************/

