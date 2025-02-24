#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>
#include <vector>
#include <project/points.h>

using namespace std;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub_custom;

// ROI 필터링
void ROI(pcl::PointCloud<pcl::PointXYZ>::Ptr sky, pcl::PointCloud<pcl::PointXYZ>::Ptr sky2) {
    for (int i = 0; i < sky->points.size(); i++) {
        pcl::PointXYZ point = sky->points[i];
        if (point.x >= 0.0 && point.x <= 44.0 && point.y >= -23.0 && point.y <= 23.0){
            if (std::sqrt(point.x * point.x + point.y * point.y) > 0.8) {
                sky2->points.push_back(point); // 조건 만족 시 저장
            }
        }
    }
}

// Voxel Grid 필터링
void voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr sky2, pcl::PointCloud<pcl::PointXYZ>::Ptr sky3) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(sky2);              // 입력
    sor.setLeafSize(0.1f, 0.1f, 0.1f);   // leaf size 10cm
    sor.filter(*sky3);
}

// 법선 벡터 내적 필터링ac_cloud,
void normalVectorFiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr sky4, pcl::PointCloud<pcl::PointXYZ>::Ptr sky5) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr wall_inliers(new pcl::PointIndices());  // 벽면 점을 추적하는 변수
    pcl::PointCloud<pcl::PointXYZ>::Ptr sky4_5(new pcl::PointCloud<pcl::PointXYZ>(*sky4));
    int max_iterations = 50; // 최대 반복 횟수
    int iteration = 0;
    
    // RANSAC 세그멘테이션 반복
    while (iteration++ < max_iterations) {
        // RANSAC 세그멘테이션 초기화
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.3);  // 허용 거리 20cm
        seg.setInputCloud(sky4);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "No plane found!" << std::endl;
            break;
        }

        // 법선 벡터를 기준으로 지면인지 벽면인지 판별
        if (!inliers->indices.empty()) { // Z축에 가까운 평면(지면)
            pcl::PointXYZ normal_vector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

            // Z축 벡터와 내적을 구해보자
            pcl::PointXYZ z_axis(0, 0, 1);
            float dot_product = normal_vector.x * z_axis.x + normal_vector.y * z_axis.y + normal_vector.z * z_axis.z;

            if (fabs(dot_product) > 0.8) {
                std::cout << "Ground plane detected, removing..." << std::endl;

                // 지면을 제거: sky4에서 해당 inliers 점들을 제거
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(sky4);
                extract.setIndices(inliers);
                extract.setNegative(true); // true로 설정하면 inliers 외 점들만 남김
                extract.filter(*sky4_5);
                // sky5에 남은 점들을 저장
                *sky5 = *sky4_5;
                break; // 지면을 찾았으므로 반복 종료
            }

            else { // 벽면을 찾았을 때
                std::cout << "Wall plane detected, skipping..." << std::endl;

                // 벽면을 제외한 점들을 wall_inliers에 저장
                *wall_inliers = *inliers;

                // 벽면 점들은 다시 RANSAC에서 제외
                pcl::ExtractIndices<pcl::PointXYZ> extractWall;
                extractWall.setInputCloud(sky4);
                extractWall.setIndices(wall_inliers);
                extractWall.setNegative(true); // 벽면을 제외
                extractWall.filter(*sky4); // 벽면 제외한 나머지 점들을 다시 sky4에 저장

                continue; // 벽면을 제외하고 계속 진행
            }
        }
    }
}

void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr sky5, pcl::PointCloud<pcl::PointXYZ>::Ptr sky6,std::vector<std::vector<float>> &pop,pcl::PointCloud<pcl::PointXYZ>::Ptr sky8){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (sky5);  //KdTree 생성
    std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장
    // 군집화 오브젝트 생성
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud (sky5);       // 입력
    ec.setClusterTolerance (0.5);  // 5cm
    ec.setMinClusterSize (10);     // 최소 포인트 수 / 풀숲에 영향을 줌
    ec.setMaxClusterSize (10000);   // 최대 포인트 수
    ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정
    ec.extract (cluster_indices);   // 군집화 적용
    cout << "Number of clusters found: " << cluster_indices.size() << std::endl;

    if (cluster_indices.size()==0) {
            cout << "No clusters found. Continuing with empty result." << std::endl;
            //예시: 기본적인 빈 포인트 추가
            pcl::PointXYZ empty_point(0.0, 0.0, 0.0);
            sky6->points.push_back(empty_point);
            pop.clear();
            return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr allClusters(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> clusterPoints; //한 군집에 속한 모든 포인트들
    std::vector<float> a1;
    int counting=0;

    for (std::vector<pcl::PointIndices>::iterator cluster_it = cluster_indices.begin(); cluster_it != cluster_indices.end(); ++cluster_it) {
        pcl::PointXYZ centroid(0.0, 0.0, 0.0);
        // 클러스터 내 모든 포인트의 평균 계산
        for (std::vector<int>::iterator point_it = cluster_it->indices.begin(); point_it != cluster_it->indices.end(); ++point_it) {
            centroid.x += sky5->points[*point_it].x;
            centroid.y += sky5->points[*point_it].y;
            centroid.z += sky5->points[*point_it].z;
            clusterPoints.push_back(sky5->points[*point_it]);
        }
        centroid.x /= cluster_it->indices.size();
        centroid.y /= cluster_it->indices.size();
        centroid.z /= cluster_it->indices.size();
        if(centroid.z<-0.6){
            continue;
        }
        // 코어 포인트를 최종 클러스터 점군에 추가
        sky6->points.push_back(centroid);
        for(int i=0;i<clusterPoints.size();i++){
            allClusters->points.push_back(clusterPoints[i]);// 'sky6랑 인덱스값이 똑같은 곳에다가 해당 군집포인트들 넣기'가 의도였지만 allClusters->points.push_back은 "하나의 점"만 추가 되므로 실패
        }
        a1.push_back(clusterPoints.size());
        clusterPoints.clear();
        counting++;

    }

    if (sky6->points.size()==0) {
            cout << "eliminating grass" << std::endl;
            //예시: 기본적인 빈 포인트 추가
            pcl::PointXYZ empty_point(0.0,0.0,0.0);
            sky6->points.push_back(empty_point);
            pop.clear();
            return;
    }

    int numClusters = cluster_indices.size();
    pop.resize(numClusters);  // 군집 수만큼 외부 벡터 크기 설정

    for (size_t i = 0; i < numClusters; ++i) {
        pop[i].resize(2);  // 각 군집에 대해 내부 벡터 크기(2) 설정 (x, y 크기)
    }

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        // 하나의 군집에 대해 min/max 값을 구하기 위해 포인트 클라우드 동적 할당
        pcl::PointCloud<pcl::PointXYZ>::Ptr sky7(new pcl::PointCloud<pcl::PointXYZ>);

        // 해당 군집에 속한 점들을 sky7에 추가
        for (size_t j = 0; j < cluster_indices[i].indices.size(); ++j) {
            int idx = cluster_indices[i].indices[j];
            sky7->points.push_back(sky5->points[idx]);  // sky6에서 해당 인덱스의 점을 추가
        }

        pcl::PointXYZ min_point, max_point;

        // sky7에서 min/max 값을 구함
        pcl::getMinMax3D(*sky7, min_point, max_point);

        // x, y 크기 계산하여 pop[i][0]과 pop[i][1]에 저장
        pop[i][0] = max_point.x - min_point.x;
        pop[i][1] = max_point.y - min_point.y;
    }
    //else{

    // 결과를 정리cmka
    sky6->width = sky6->points.size();
    sky6->height = 1;
    sky6->is_dense = true;

    int o_x;
    for (int i=0;i<sky6->points.size();i++){
        o_x=0;
        for(int j=i+1;j<sky6->points.size();j++){
            if(std::sqrt((sky6->points[i].x-sky6->points[j].x)*(sky6->points[i].x-sky6->points[j].x) + (sky6->points[i].y-sky6->points[j].y)*(sky6->points[i].y-sky6->points[j].y))<=10){
                pcl::PointCloud<pcl::PointXYZ>::Ptr sky6_8(new pcl::PointCloud<pcl::PointXYZ>);//sky6_8은 해당 거리값 이내에 있는 코어포인트 합치는 용도
                int i_n=0;
                int j_n=0;
                for(int u=0;u<i;u++){
                    i_n+=a1[u];
                }

                for(int u=0;u<j;u++){
                    j_n+=a1[u];
                }

                for(int u=i_n;u<=i_n+a1[i];u++){
                    sky6_8->points.push_back(allClusters->points[i_n]);
                }

                for(int u=j_n;u<=j_n+a1[j];u++){
                    sky6_8->points.push_back(allClusters->points[j_n]);
                }

                pcl::PointXYZ new_centroid(0.0, 0.0, 0.0);
                for(int w=0;w<sky6_8->points.size();w++){
                    new_centroid.x += sky6_8->points[w].x;
                    new_centroid.y += sky6_8->points[w].y;
                    new_centroid.z += sky6_8->points[w].z;
                }

                new_centroid.x /=sky6_8->points.size();
                new_centroid.y /=sky6_8->points.size();
                new_centroid.z /=sky6_8->points.size();

                sky8->points.push_back(new_centroid);
                o_x=1;
                break;
                //중심점 구하고 추가후 반복문 나오기
            }

        }

        if(o_x==0){
            sky8->points.push_back(sky6->points[i]);
        }
    }
}

// 콜백 함수
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg) {
    // 1. ROS 메시지를 PCL 데이터로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boxeled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustering_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    ROI(cloud, filtered_cloud);  // ROI 필터링 적용
    // ROS_INFO("ROI Filtering Completed");

    // 3. 복셀화 (Voxel Grid)
    voxel(filtered_cloud, boxeled_cloud);
    // ROS_INFO("Voxel Filtering Completed");

    // 5. 벡터 내적 필터링 (법선 벡터와 Z축 내적을 통해 지면 추출 및 제거)
    normalVectorFiltering(boxeled_cloud, ransac_cloud);
    // ROS_INFO("Normal Vector Filtering Completed");

    std::vector<std::vector<float>>xy;
    clustering(ransac_cloud,clustering_cloud,xy,final_cloud);
    ROS_INFO("CLustering Completed");

    // 6. PCL 데이터를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*final_cloud, output_msg);
    output_msg.header = input_msg->header;  // 입력 메시지의 헤더를 유지

    sensor_msgs::PointCloud2 output_msg2;
    pcl::toROSMsg(*ransac_cloud, output_msg2);
    output_msg2.header = input_msg->header;  // 입력 메시지의 헤더를 유지

    // ROS_INFO("Received point cloud data");

    // 7. 필터링된 데이터를 발행
    pub.publish(output_msg);
    pub2.publish(output_msg2);

    if(!(clustering_cloud->points[0].x==0&&clustering_cloud->points[0].y==0&&clustering_cloud->points[0].z==0)){
        project::points cloud_points;

        // 클러스터의 각 점을 cloud_points.data에 저장하고 idx 값을 설정합니다.
        for (int i = 0; i < clustering_cloud->size(); i++) {
            project::point tmp;
            tmp.x = clustering_cloud->points[i].x;
            tmp.y = clustering_cloud->points[i].y;
            tmp.z = clustering_cloud->points[i].z;
            tmp.distance= sqrt(tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z);
            tmp.degree=atan2(tmp.y,tmp.x)*180.0/M_PI;
            tmp.x_size=xy[i][0];
            tmp.y_size=xy[i][1];
            if(tmp.degree<=15&&tmp.degree>=-15){
                tmp.object="Caution point";
            }
            cloud_points.data.push_back(tmp);
        }

        // 점들을 거리 순으로 정렬 (버블 정렬)
        for (int i = 0; i < cloud_points.data.size() - 1; i++) {
            for (int j = 0; j < cloud_points.data.size() - 1 - i; j++) {
                // 두 점의 거리 계산
                double distance1 = cloud_points.data[j].x * cloud_points.data[j].x +
                       cloud_points.data[j].y * cloud_points.data[j].y +
                       cloud_points.data[j].z * cloud_points.data[j].z;

                double distance2 = cloud_points.data[j + 1].x * cloud_points.data[j + 1].x +
                       cloud_points.data[j + 1].y * cloud_points.data[j + 1].y +
                       cloud_points.data[j + 1].z * cloud_points.data[j + 1].z;
                // 거리 비교 후 점 스왑
                if (distance1 > distance2) {
                    project::point temp = cloud_points.data[j];
                    cloud_points.data[j] = cloud_points.data[j + 1];
                    cloud_points.data[j + 1] = temp;
                }
            }
        }

        cloud_points.s=cloud_points.data.size();

        cloud_points.num="number of total core points";

        //거리값 계산이랑 roi y=0인 구간에서 발견된 코어 포인트 따로 정리

        pub_custom.publish(cloud_points);
    }

    else{
        project::points cloud_points;
        project::point tmp;
        cloud_points.s=0;
        cloud_points.num="number of total core points";
        tmp.x = 0;
        tmp.y = 0;
        tmp.z = 0;
        tmp.index = 0;  // 원래의 인덱스를 idx로 저장합니다.
        tmp.distance=0;
        tmp.degree=0;
        tmp.x_size=0;
        tmp.y_size=0;
        pub_custom.publish(cloud_points);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "morai_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    // Subscriber: PointCloud2 입력
    ros::Subscriber sub = nh.subscribe("/lidar3D", 1000, pointCloudCallback);

    // Publisher: 필터링된 PointCloud2 출력
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_lidar3D", 10);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/ransac_lidar3D", 10);
    pub_custom = nh.advertise<project::points>("/lidar3d_custom", 10);

    // ros::spin();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep(); // 30zh 맞췄는데 8hz  나옴
    }
    return 0;

}