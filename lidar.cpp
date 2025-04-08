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
#include <Eigen/Dense>
#include <filter/points.h>
#include <sensor_msgs/Imu.h>

using namespace std;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub_custom;

const int MAX_FRAMES = 5;
int speed_counter=0; //속도값 갯수
//전프레임의 시간
ros::Time prevTime;
//현프레임의 시간
ros::Time currTime;

struct objif {
    pcl::PointIndices* objPoints;
    float core_x;
    float core_y;
    float core_z;
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;

    float velocity;
    float acceleration;

    float dir_x;
    float dir_y;
};
std::deque<std::vector<objif>> prev_objects_queue;

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
    int max_iterations = 100; // 최대 반복 횟수
    int iteration = 0;
    
    // RANSAC 세그멘테이션 반복
    while (iteration++ < max_iterations) {
        // RANSAC 세그멘테이션 초기화
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.5);  // 허용 거리 20cm
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
                extract.setInputCloud(sky4_5);
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

void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr sky5, pcl::PointCloud<pcl::PointXYZ>::Ptr sky6,
std::vector<std::vector<float>> &pop,std::vector<pcl::PointIndices> &all_cluster_indices ){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (sky5);  //KdTree 생성
    std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장
    // 군집화 오브젝트 생성
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud (sky5);       // 입력
    ec.setClusterTolerance (2);  // 5cm
    ec.setMinClusterSize (10);     // 최소 포인트 수 / 풀숲에 영향을 줌
    ec.setMaxClusterSize (10000);   // 최대 포인트 수
    ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정
    ec.extract (cluster_indices);   // 군집화 적용
    cout << "Number of clusters found: " << cluster_indices.size() << std::endl;
    all_cluster_indices=cluster_indices;
    if (cluster_indices.size()==0) {
            cout << "No clusters found. Continuing with empty result." << std::endl;
            //예시: 기본적인 빈 포인트 추가
            pcl::PointXYZ empty_point(0.0, 0.0, 0.0);
            sky6->points.push_back(empty_point);
            pop.clear();
            return;
    }
    

    for (std::vector<pcl::PointIndices>::iterator cluster_it = cluster_indices.begin(); cluster_it != cluster_indices.end(); ++cluster_it) {
        pcl::PointXYZ centroid(0.0, 0.0, 0.0);
        // 클러스터 내 모든 포인트의 평균 계산
        for (std::vector<int>::iterator point_it = cluster_it->indices.begin(); point_it != cluster_it->indices.end(); ++point_it) {
            centroid.x += sky5->points[*point_it].x;
            centroid.y += sky5->points[*point_it].y;
            centroid.z += sky5->points[*point_it].z;
            
        }
        centroid.x /= cluster_it->indices.size();
        centroid.y /= cluster_it->indices.size();
        centroid.z /= cluster_it->indices.size();
        //if(centroid.z<-0.6){
            //continue;
        //}// 풀 같은 지면제거 후에 남은 노이즈 제거하기 위한 조건문
        // 코어 포인트를 최종 클러스터 점군에 추가 // 현재 병합과정으로 인해 삭제함
        sky6->points.push_back(centroid);

    }

    if (sky6->points.size()==0) {
            cout << "eliminating grass" << std::endl;
            //예시: 기본적인 빈 포인트 추가
            pcl::PointXYZ empty_point(0.0,0.0,0.0);
            sky6->points.push_back(empty_point);
            pop.clear();
            return;
    }
    int counting=0;
    int numClusters = cluster_indices.size();
    pop.resize(numClusters);  // 군집 수만큼 외부 벡터 크기 설정

    for (size_t i = 0; i < numClusters; ++i) {int counting=0;
        pop[i].resize(2);  // 각 군집에 대해 내부 벡터 크기(2) 설정 (x, y 크기)
    }

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        // 하나의 군집에 대해 min/max 값을 구하기 위해 포인트 클라우드 동적 할당
        pcl::PointCloud<pcl::PointXYZ>::Ptr sky6_6(new pcl::PointCloud<pcl::PointXYZ>);

        // 해당 군집에 속한 점들을 sky6_6에 추가
        for (size_t j = 0; j < cluster_indices[i].indices.size(); ++j) {
            int idx = cluster_indices[i].indices[j];
            sky6_6->points.push_back(sky5->points[idx]);  // sky6에서 해당 인덱스의 점을 추가
        }

        pcl::PointXYZ min_point, max_point;

        // 6_6에서 min/max 값을 구함
        pcl::getMinMax3D(*sky6_6, min_point, max_point);

        // x, y 크기 계산하여 pop[i][0]과 pop[i][1]에 저장
        pop[i][0] = max_point.x - min_point.x;
        pop[i][1] = max_point.y - min_point.y;
    }
    //else{

    // 결과를 정리cmka
    sky6->width = sky6->points.size();
    sky6->height = 1;
    sky6->is_dense = true; 
}

void merge(std::vector<objif> &ob , pcl::PointCloud<pcl::PointXYZ>::Ptr sky5,int d){
    const float DISTANCE = 4;
    bool merged;
        while(merged){
            merged=false;
            for(std::vector<objif>::iterator it=ob.begin();it != ob.end() ; ++it){
                for(std::vector<objif>::iterator it2=it+1;it2 != ob.end();){
                    bool shouldMerge=false;
                    float dx=(*it).core_x - (*it2).core_x;
                    float dy=(*it).core_y - (*it2).core_y;
                    float dist=sqrt(dx*dx+dy*dy);
                    if(dist<= DISTANCE){
                        const std::vector<int> &indices1 =(*it).objPoints->indices;
                        const std::vector<int> &indices2 =(*it2).objPoints->indices;
                        const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &points = sky5->points;
                        for(int i : indices1){
                            const pcl::PointXYZ &PP=points[i];
                            for(int i2 : indices2){
                                const pcl::PointXYZ &PP2=points[i2];
                                float pdx = PP.x - PP2.x;
                                float pdy = PP.y - PP2.y;
                                if ((pdx * pdx + pdy * pdy) <= (d * d)) {
                                    shouldMerge = true;
                                    break;
                                }
                            }
                            if(shouldMerge) break;
                        }
                    }
                    if(shouldMerge)
                    {
                        it->xMin = std::min(it->xMin, it2->xMin);
                        it->yMin = std::min(it->yMin, it2->yMin);
                        it->zMin = std::min(it->zMin, it2->zMin);
                        it->xMax = std::max(it->xMax, it2->xMax);
                        it->yMax = std::max(it->yMax, it2->yMax);
                        it->zMax = std::max(it->zMax, it2->zMax);
                        int totalPoints = it->objPoints->indices.size() +it2->objPoints->indices.size();
                        float sumX = 0, sumY = 0, sumZ = 0;
                        for (int i : it->objPoints->indices) {
                            sumX += sky5->points[i].x;
                            sumY += sky5->points[i].y;
                            sumZ += sky5->points[i].z;
                        }
                        for (int i : it2->objPoints->indices) {
                            sumX += sky5->points[i].x;
                            sumY += sky5->points[i].y;
                            sumZ += sky5->points[i].z;
                        }
                        it->core_x = sumX / totalPoints;
                        it->core_y = sumY / totalPoints;
                        it->core_z = sumZ / totalPoints;

                        // 포인트 인덱스 병합
                        (*it).objPoints->indices.insert((*it).objPoints->indices.end(), 
                                                     (*it2).objPoints->indices.begin(), 
                                                     (*it2).objPoints->indices.end());

                        it2 = ob.erase(it2); // 벡터에서 삭제 후 다음 클러스터로
                        merged = true; // 병합이 일어났음을 표시
                        break;
                    } else{
                        ++it2;
                    }
                }
                if (merged) break;
            }
            
        }
        //std::cout<<"completed"<<endl;
    
    
}


/*void kalman_filter_update(objif &obj, float measured_x, float measured_y, float dt) {
    const int state_size = 4;  // [x, y, vx, vy]
    const int meas_size = 2;   // [x, y]

    // 상태 전이 행렬 At
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(state_size, state_size);
    A(0, 2) = dt;  // x += vx * dt
    A(1, 3) = dt;  // y += vy * dt

    // 측정 행렬 H
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(meas_size, state_size);
    H(0, 0) = 1;  // x 측정값 사용
    H(1, 1) = 1;  // y 측정값 사용

    // 프로세스 잡음 공분산 Q
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(state_size, state_size) * 0.01f;

    // 측정 잡음 공분산 R
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(meas_size, meas_size) * 0.1f;

    // 측정값 z
    Eigen::VectorXf z(meas_size);
    z << measured_x, measured_y;

    // Kalman 필터 상태 벡터 초기화
    if (!obj.kf_initialized) {
        obj.kf_state = Eigen::VectorXf::Zero(state_size);
        obj.kf_state.head(2) = z;  // x, y
        obj.kf_P = Eigen::MatrixXf::Identity(state_size, state_size);
        obj.kf_initialized = true;
        return;
    }

    // 예측 단계
    Eigen::VectorXf x_pred = A * obj.kf_state;
    Eigen::MatrixXf P_pred = A * obj.kf_P * A.transpose() + Q;

    // 업데이트 단계
    Eigen::VectorXf y = z - H * x_pred;
    Eigen::MatrixXf S = H * P_pred * H.transpose() + R;
    Eigen::MatrixXf K = P_pred * H.transpose() * S.inverse();

    obj.kf_state = x_pred + K * y;
    obj.kf_P = (Eigen::MatrixXf::Identity(state_size, state_size) - K * H) * P_pred;

    // 결과 반영
    obj.core_x = obj.kf_state(0);
    obj.core_y = obj.kf_state(1);

    float vx = obj.kf_state(2);
    float vy = obj.kf_state(3);
    obj.velocity = std::sqrt(vx * vx + vy * vy);
    obj.dir_x = vx / (obj.velocity + 1e-6f);  // 0 나눗셈 방지용
    obj.dir_y = vy / (obj.velocity + 1e-6f);
}*/



/*void objectinfo_save(std::vector<objif> &object,pcl::PointCloud<pcl::PointXYZ>::Ptr sky5,pcl::PointCloud<pcl::PointXYZ>::Ptr sky6,
std::vector<pcl::PointIndices> all_cluster_indices,pcl::PointCloud<pcl::PointXYZ>::Ptr sky7,
std::vector<objif> &prev_objects){
    pair<float, float> x(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); 
    //x.first에는 최솟값을, x.second에는 최댓값을 저장
    pair<float,float>y(std::numeric_limits<float>::max(),std::numeric_limits<float>::lowest());
    pair<float, float> z(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); 
    int centroid_index =0;

    for(std::vector<pcl::PointIndices>::iterator it = all_cluster_indices.begin();it != all_cluster_indices.end(); ++it){
        for(std::vector<int>::iterator it2 = it->indices.begin();it2 != it->indices.end();++it2){
            if(x.first>sky5->points[*it2].x) x.first=sky5->points[*it2].x;
            if(x.second<sky5->points[*it2].x) x.second=sky5->points[*it2].x;
            if(y.first>sky5->points[*it2].y) y.first=sky5->points[*it2].y;
            if(y.second<sky5->points[*it2].y) y.second=sky5->points[*it2].y;
            if(z.first>sky5->points[*it2].z) z.first=sky5->points[*it2].z;
            if(z.second<sky5->points[*it2].z) z.second=sky5->points[*it2].z;
        }
        
        
        objif before_ob={&(*it),sky6->points[centroid_index].x,sky6->points[centroid_index].y,sky6->points[centroid_index].z,
                           x.first,y.first,z.first,x.second,y.second,z.second };

        object.push_back(before_ob);
        centroid_index++;
    }
    
    if(object.size()>=2){
        //merge(object,sky5,3);
        for(int i=0;i<object.size();i++){
            pcl::PointXYZ core;
            core.x=object[i].core_x;
            core.y=object[i].core_y;
            core.z=object[i].core_z;
            sky7->points.push_back(core);
        }
    }
    else{
        pcl::PointXYZ core={1,1,1};
        sky7->points.push_back(core);
    }
}*/


void objectinfo_for_km(std::vector<objif> &now_object,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &sky5,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &sky6,
                        std::vector<pcl::PointIndices> all_cluster_indices) {
    
    now_object.clear();
    int centroid_index = 0;

    for (int i = 0; i < all_cluster_indices.size(); i++) {
        pcl::PointIndices cluster = all_cluster_indices[i];

        float xMin = std::numeric_limits<float>::max();
        float yMin = std::numeric_limits<float>::max();
        float zMin = std::numeric_limits<float>::max();
        float xMax = std::numeric_limits<float>::lowest();
        float yMax = std::numeric_limits<float>::lowest();
        float zMax = std::numeric_limits<float>::lowest();

        for (int j = 0; j < cluster.indices.size(); j++) {
            int idx = cluster.indices[j];
            const pcl::PointXYZ &pt = sky5->points[idx];
            xMin = std::min(xMin, pt.x);
            yMin = std::min(yMin, pt.y);
            zMin = std::min(zMin, pt.z);
            xMax = std::max(xMax, pt.x);
            yMax = std::max(yMax, pt.y);
            zMax = std::max(zMax, pt.z);
        }

        pcl::PointXYZ centroid = sky6->points[centroid_index];
        centroid_index++;

        objif current_obj;
        current_obj.objPoints = &all_cluster_indices[i];
        current_obj.core_x = centroid.x;
        current_obj.core_y = centroid.y;
        current_obj.core_z = centroid.z;
        current_obj.xMin = xMin;
        current_obj.yMin = yMin;
        current_obj.zMin = zMin;
        current_obj.xMax = xMax;
        current_obj.yMax = yMax;
        current_obj.zMax = zMax;
        current_obj.velocity = 0.0f;
        current_obj.acceleration = 0.0f;
        current_obj.dir_x = 0.0f;
        current_obj.dir_y = 0.0f;
        now_object.push_back(current_obj);
    }
}


double calculateSpeed(const std::vector<objif>  &past_objects,int a, const std::vector<objif> &now_object,int b, double timeInterval) {
    double dx=now_object[b].core_x - past_objects[a].core_x;
    double dy=now_object[b].core_y - past_objects[a].core_y;
    double distance = std::sqrt(dx * dx + dy * dy );// 두 점 사이의 거리
    return distance / timeInterval;  // 속도 계산 (거리 / 시간)
}

void direction(const std::vector<objif>  &past_objects,int a, std::vector<objif> &now_object,int b) {
    
    double dx=now_object[b].core_x - past_objects[a].core_x;
    double dy=now_object[b].core_y - past_objects[a].core_y;
    double distance=std::sqrt(dx * dx + dy * dy );
    double dir_x = (now_object[b].core_x - past_objects[a].core_x)/distance;
    double dir_y = (now_object[b].core_y - past_objects[a].core_y)/distance;
    cout<<"direction of x: "<<dir_x<<endl;
    cout<<"direction of y: "<<dir_y<<endl;
}

// 전/현재 프레임 군집 중심점 비교 후 속도 가속도 방향 구하는 함수
void compareCentroids(const std::deque<std::vector<objif>> &past_objects_queue,
                      std::vector<objif> &now_object,double time) {

    const std::vector<objif> &compare=past_objects_queue.back();

    for (size_t i = 0; i < compare.size(); ++i) {
        for (size_t j = 0; j < now_object.size(); ++j) {
            double dx=now_object[i].core_x - compare[j].core_x;
            double dy=now_object[i].core_y - compare[j].core_y;
            double distance = std::sqrt(dx * dx + dy * dy );

            // 5cm 이내일 경우 같은 군집으로 판단하고 속도 계산
            if (distance < 5) {  // 5cm 이내 but 이거는 잘 안나오면 조절해야 되는 파라 미터
                double speed = calculateSpeed(compare,j, now_object,i, time);  // 0.1초 간격
                now_object[i].velocity=speed;
                std::cout<<"Speed:"<<speed<<endl;
                speed_counter++;
                if(speed_counter>=2){
                    double accelation= fabs(now_object[i].velocity- compare[j].velocity) / time ; 
                    std::cout<<"Accelation:"<<accelation<<endl;
                }
                
                direction(compare,i, now_object,j);

            }
        }
    }
}


// 두 군집 중심점 간의 속도 계산 함수



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
    // ROS_INFO("Voxel Filtering Completed");clustering_cloud
    normalVectorFiltering(boxeled_cloud, ransac_cloud);
    std::vector<std::vector<float>>xy;
    std::vector<pcl::PointIndices> all_cluster_indices;
    clustering(ransac_cloud,clustering_cloud,xy,all_cluster_indices);
    ROS_INFO("CLustering Completed");

    std::vector<objif> object;
    double time = (currTime - prevTime).toSec();
    std::cout << "Time interval between frames: " << (currTime - prevTime).toSec() << " seconds" << std::endl;
    
    objectinfo_for_km(object,ransac_cloud,clustering_cloud,all_cluster_indices);

    if (!prev_objects_queue.empty()) {
        double time=(currTime-prevTime).toSec();
        compareCentroids(prev_objects_queue,object,time);
    }

    prev_objects_queue.push_back(object);
    prevTime = currTime;
    currTime = input_msg->header.stamp;

    if (prev_objects_queue.size() > MAX_FRAMES) {
        prev_objects_queue.pop_front();  // 가장 오래된 프레임 제거
    }


    // 6. PCL 데이터를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*clustering_cloud, output_msg);
    output_msg.header = input_msg->header;  // 입력 메시지의 헤더를 유지

    sensor_msgs::PointCloud2 output_msg2;
    pcl::toROSMsg(*ransac_cloud, output_msg2);
    output_msg2.header = input_msg->header;  // 입력 메시지의 헤더를 유지
 
    // ROS_INFO("Received point cloud data");

    // 7. 필터링된 데이터를 발행
    pub.publish(output_msg);
    pub2.publish(output_msg2);

    if(!(clustering_cloud->points[0].x==0&&clustering_cloud->points[0].y==0&&clustering_cloud->points[0].z==0)){
        filter::points cloud_points;

        // 클러스터의 각 점을 cloud_points.data에 저장하고 idx 값을 설정합니다.
        for (int i = 0; i < clustering_cloud->size(); i++) {
            filter::point tmp;
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
                    filter::point temp = cloud_points.data[j];
                    cloud_points.data[j] = cloud_points.data[j + 1];
                    cloud_points.data[j + 1] = temp;
                }
            }
        }

        cloud_points.s=cloud_points.data.size();

        cloud_points.num="number of total core points";

        pub_custom.publish(cloud_points);
    }

    else{
        filter::points cloud_points;
        filter::point tmp;
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
    pub_custom = nh.advertise<filter::points>("/lidar3d_custom", 10);
    
    // ros::spin();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep(); // 30zh 맞췄는데 8hz  나옴
    }
    return 0;

}

