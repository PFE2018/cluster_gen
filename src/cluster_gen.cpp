#include <ros/ros.h>

#include <cluster_gen.h>




#define _USE_MATH_DEFINES


/**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **				    			  Classe ClusterClass				          	 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/

ClusterClass::ClusterClass(float coor_x, float coor_y, float coor_z,
		float radius_of_curv, float init_steer_angle) {

	cluster_width_ = 0.1;   //Hardcode pour l'instant

	cluster_height_ = 0.1;	//Hardcode pour l'instant

	validation_nb_epoq_ = 2;  	//Hardcode pour l'instant

	//Init de la position du cluster
	x_ = coor_x;
	y_ = coor_y;
	z_ = coor_z;

	x_moy_ = coor_x;
	y_moy_ = coor_y;
	z_moy_ = coor_z;

	//Variable de test pour gestion des clusters
	nb_point_actu_ = 1;
	fault_ = 1;
	point_count_ = 1;
	epoq_count_ = 1;
    fault_epoq_count_=0;



}

//Indique si un point fait partie ou non du cluster
int ClusterClass::is_a_member(float point_x, float height, float depth) {

	if (point_x > (x_ - cluster_width_/2) && point_x < (x_ + cluster_width_/2)
			&& height > (y_ - cluster_height_ / 2)
			&& height < (y_ + cluster_height_/2) && depth > (z_ - cluster_width_/2)
			&& depth < (z_ + cluster_width_/2)) {

		point_count_ += 1;

		fault_ = 1; //Signifie qu'il y a un point en erreur dans le frame du point cloud actuel

		return 1; //utile pour faire une boucle qui permet de voir si le point appartient a un cluster  voir TREATMENT::treat

	}

	return 0;
}


/**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **						      Classe TreatmentClass					          	 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/


//Initialisation des paramètres de la classe de traitement
void TreatmentClass::init(ros::NodeHandle& nh) {

	int id_number=1;

    //Publisher
	pub_rviz_marker_msg = nh.advertise<adas_msgs::cluster_visualiser>(
			"rviz_marker", 1);

    pub_eigenvalues= nh.advertise<geometry_msgs::Point>(
            "pcl_eigenvalues", 1);

    pub_centroid= nh.advertise<geometry_msgs::Point>(
            "centroid_XYZ", 1);


	//Subcribers
	sub_kinect_cloud = nh.subscribe("/filtered_pcloud", 1,
			&TreatmentClass::pcl_cloud_cb, this);

	rviz_marker_msg.header.frame_id = "kinect2_ir_optical_frame";
}

void TreatmentClass::treat(const sensor_msgs::PointCloud2ConstPtr& input) {

	/********************************************************************************

	 Effectue le traitement sur le point_cloud d'intérêt ouvre les cluster
	 et gère le signaux de controle envoyé aux autres noeuds

	 *********************************************************************************/

	//**********************************Section 1**********************************
	pcl::PCLPointCloud2::Ptr pcl2(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZ(
			new pcl::PointCloud<pcl::PointXYZ>);


	int nb_clust = 0;

	//Convertion du point_coud

	// Convert to PCL data type
	pcl_conversions::toPCL(*input, *pcl2);

	// Fill in the cloud data
	pcl::fromPCLPointCloud2(*pcl2, *pclXYZ); //(convertion entre sensor_msgs::PointCloud2 et  pcl::PointCloud<pcl::PointXYZ>.

	pclXYZ->header.frame_id = "kinect2_ir_optical_frame";

	//Boucle pour réinitialiser certains indices de la classe curv_cluster
	for (std::list<ClusterClass>::iterator it = curv_cluster_.begin();
			it != curv_cluster_.end(); it++) {

		it->x_moy_ = it->x_;
		it->y_moy_ = it->y_;
		it->z_moy_ = it->z_;

		it->fault_ = 0;

		it->nb_point_actu_ = 1;

	}


//
//        pcl::IndicesPtr test;
//        test=extract.getIndices();




	rviz_marker_msg.coor_x.clear();
	rviz_marker_msg.coor_y.clear();
	rviz_marker_msg.coor_z.clear();
	rviz_marker_msg.type.clear();
	rviz_marker_msg.nb_pts = 0;



	//**********************************Section 2**********************************

	//Boucle de test pour trouver le points en fautes et le point le plus proche + gestion des clusters
	for (size_t k = 0; k < pclXYZ->points.size(); ++k) {

		int is_in = 0; //Indice pour déterminer si le point du point_cloud appartient à un cluster existant


		cluster_indices.push_back(k);

				for (std::list<ClusterClass>::iterator it =
						curv_cluster_.begin(); it != curv_cluster_.end();
						it++) {

					if (it->is_a_member(pclXYZ->points[k].x,
							pclXYZ->points[k].y, pclXYZ->points[k].z) == 1) {

						it->x_moy_ += pclXYZ->points[k].x;
						it->y_moy_ += pclXYZ->points[k].y;
						it->z_moy_ += pclXYZ->points[k].z;

						it->nb_point_actu_++;
						is_in++;
						it->fault_ = 1;

					}

				}

				if (is_in == 0) {
					ClusterClass nouveau(pclXYZ->points[k].x,
							pclXYZ->points[k].y, pclXYZ->points[k].z,
							radius_of_curv_, steering_angle_);


					curv_cluster_.push_back(nouveau);
				}

			}







	pcl::compute3DCentroid(*pclXYZ,cluster_indices,centroid);

	// Compute the 3x3 covariance matrix
	pcl::computeCovarianceMatrix (*pclXYZ, centroid, covariance_matrix);
	pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_value);
	std::cout << "centroid-x:"<<centroid[0]<<"centroid-y:"<<centroid[1]<<"centroid-z:"<<centroid[2]<<std::endl;

    centroid_point.x=centroid[0];
    centroid_point.y=centroid[1];
    centroid_point.z=centroid[2];

    eigen_1=eigen_value.x();
    eigen_2=eigen_value.y();
    eigen_3=eigen_value.z();

    eigenvals.x=eigen_1;
    eigenvals.y=eigen_2;
    eigenvals.z=eigen_3;



    pub_eigenvalues.publish(eigenvals);

    pub_centroid.publish(centroid_point);



    rviz_marker_msg.coor_x.push_back(centroid[0]);
    rviz_marker_msg.coor_y.push_back(centroid[1]);
    rviz_marker_msg.coor_z.push_back(centroid[2]);
    rviz_marker_msg.type.push_back(0);

    nb_clust++;



	rviz_marker_msg.coor_x.push_back(centroid[0]);
	rviz_marker_msg.coor_y.push_back(centroid[1]);
	rviz_marker_msg.coor_z.push_back(centroid[2]);
	rviz_marker_msg.type.push_back(0);

	nb_clust++;

	cluster_indices.clear();


	//**********************************Section 3**********************************



	// On crée le message pour visualisation sur rviz puis on efface les clusters jugés expirés

	for (std::list<ClusterClass>::iterator it = curv_cluster_.begin();
			it != curv_cluster_.end(); it++) {

		if (it->fault_ == 0) {



			it->x_moy_ = it->x_moy_ / it->nb_point_actu_;
			it->y_moy_ = it->y_moy_ / it->nb_point_actu_;
			it->z_moy_ = it->z_moy_ / it->nb_point_actu_;

			it->x_ = it->x_moy_;
			it->y_=it->y_moy_;
			it->z_ = it->z_moy_;


			it->error_type_ = 0;

            it->fault_epoq_count_++;




				}

		if (it->fault_ == 1) {

			it->last_fault_ = it->epoq_count_;

			//Le cluster reste
			it->x_moy_ = it->x_moy_ / it->nb_point_actu_;
			it->y_moy_ = it->y_moy_ / it->nb_point_actu_;
			it->z_moy_ = it->z_moy_ / it->nb_point_actu_;

			it->error_type_ = 1;

            it->epoq_count_++;



		}


		//**********************************Section 4**********************************

		//On regarde de quel type de cluster il s'agit
		//type d'erreur pour visualisation

}



	for (std::list<ClusterClass>::iterator it = curv_cluster_.begin();
			it != curv_cluster_.end(); it++) {
		//Si le cluster est ouvert depuis 4 frame et qu'il n'est pas en faute actuellement, on envoie un flag pour dire qu'il peut être fermé

		if (it->error_type_ == 1) {
			//On crée un marker pour le cluster
			//On ajoute ce marker à la liste soumise au noeud de visualisation
			rviz_marker_msg.coor_x.push_back(it->x_moy_);
			rviz_marker_msg.coor_y.push_back(it->y_moy_);
			rviz_marker_msg.coor_z.push_back(it->z_moy_);
			rviz_marker_msg.type.push_back(it->error_type_);

			nb_clust++;



		}



		if (it->error_type_ == 0) {

			//On affiche un marker pour le cluster une dernière fois
			rviz_marker_msg.coor_x.push_back(it->x_);
			rviz_marker_msg.coor_y.push_back(it->y_);
			rviz_marker_msg.coor_z.push_back(it->z_);
			rviz_marker_msg.type.push_back(it->error_type_);

			//on efface le cluster et on incrémente l'itérateur sur le prochain élément de la liste
			it = curv_cluster_.erase(it);
		}


	}



	rviz_marker_msg.nb_pts = (nb_clust);
	rviz_marker_msg.coor_x.reserve(nb_clust);
	rviz_marker_msg.coor_y.reserve(nb_clust);
	rviz_marker_msg.coor_z.reserve(nb_clust);
	rviz_marker_msg.type.reserve(nb_clust);
	rviz_marker_msg.nb_pts = (nb_clust);

	pub_rviz_marker_msg.publish(rviz_marker_msg);

	rviz_marker_msg.coor_x.clear();
	rviz_marker_msg.coor_y.clear();
	rviz_marker_msg.coor_z.clear();
	rviz_marker_msg.type.clear();


	pos_array_.data.clear();

}

void TreatmentClass::pcl_cloud_cb(
		const sensor_msgs::PointCloud2ConstPtr& input) {

	TreatmentClass::treat(input);
}



/**********************************************MAIN*****************************************/
int main(int argc, char** argv) {

	ros::init(argc, argv, "POINTS_SAMPLING");

	ros::NodeHandle nh;

	TreatmentClass treat;

	treat.init(nh);

	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

	// Spin
	//ros::spin();

}

