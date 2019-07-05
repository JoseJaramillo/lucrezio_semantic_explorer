#include "semantic_explorer.h"


void serializeRays(const Vector3fPairVector& rays, const std::string& filename);

SemanticExplorer::SemanticExplorer(){
  _camera_pose.setIdentity();
  _objects.clear();
  _processed.clear();
  _N=4;         //Number of NBV candidates
  _radius=1.0;
}

void SemanticExplorer::setObjects(const ObjectPtrVector& semantic_map){
  _objects.clear();
  for(size_t i=0; i<semantic_map.size(); ++i){
    const ObjectPtr& o = semantic_map[i];
    const std::string& model = o->model();

    if(model == "salt" || model == "milk" || model == "tomato_sauce" || model == "zwieback")
      continue;

    //check if the object has been already processed
    StringVector::iterator it = std::find (_processed.begin(),_processed.end(),model);
    if(it!=_processed.end())
      continue;

    _objects.insert(std::make_pair(model,o));
  }
}

bool SemanticExplorer::findNearestObject(ObjectPtr &nearest_object){
  float min_dist = std::numeric_limits<float>::max();
  bool found=false; 
  for(StringObjectPtrMap::iterator it=_objects.begin(); it!=_objects.end(); ++it){     //change to rbegin and rend
    const ObjectPtr& o = it->second;
    
    //check if the object has been already processed
    StringVector::iterator itt = std::find (_processed.begin(),_processed.end(),o->model());
    if(itt!=_processed.end()){
      throw std::runtime_error("[SemanticExplorer][findNearestObject]: you're messing up things!");
      continue;
    }

    float dist=(o->position()-_camera_pose.translation()).norm();
    if(dist<min_dist){
      min_dist=dist;
      nearest_object=o;
      found=true;
    }
  }
  return found;
}

bool SemanticExplorer::findObject(const std::string objectName,ObjectPtr &nearest_object){
  float min_dist = std::numeric_limits<float>::max();
  bool found=false;
  std::cerr << "Objects in SM:" << std::endl;//
  for(StringObjectPtrMap::iterator it=_objects.begin(); it!=_objects.end(); ++it){
    const ObjectPtr& o = it->second;

    float dist=(o->position()-_camera_pose.translation()).norm();
    if(o->model()==objectName){
      nearest_object=o;
      found= true;
    }
    /*DEBUG delete after*/

    std::cerr << o->model() << "TimeStamp: " << o->ocupancy_volume() <<std::endl;

  }
  return found;
}



Isometry3fVector SemanticExplorer::generateCandidateViews(const ObjectPtr& nearest_object){
  if(!nearest_object)
    throw std::runtime_error("[SemanticExplorer][computePoses]: no nearest object!");

  Isometry3fVector candidate_views;
  for(int i=0; i<_N; i++){
    float alpha=i*(2*M_PI/(float)_N);
    float x=_radius*cos(alpha);
    float y=_radius*sin(alpha);
    float theta=atan2(-y,-x);

//    Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
//    T.translation() = Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,0.6);
//    T.linear() = Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ()).matrix();

    Eigen::Isometry3f T = v2t(Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,theta));

    candidate_views.push_back(T);
  }

  return candidate_views;
}

Isometry3fVector SemanticExplorer::generateCandidateViews_Jose(const ObjectPtr& nearest_object){
  if(!nearest_object){
    throw std::runtime_error("[SemanticExplorer][computePoses]: no nearest object!");
  }
  
  Eigen::Vector3f squaredDistances;
  float OFFSET = 0.1;
  float CLEARANCE = 0.6;
  float X=0;
  float Y=0;
  float Z=0;

  //--------------SECTION 3.2.1-----------------//
  if(true){
    if(nearest_object->model()=="fountain_park_1"){
      std::cout << "chuckles i'm in danger " << nearest_object->model() << std::endl;
      float X=4;
      float Y=4;
      float Z=3;
    }
    else if(nearest_object->model()=="lamp_street_1"){
      std::cout << "chuckles i'm in danger " << nearest_object->model() << std::endl;
      float X=0.3;
      float Y=0.3;
      float Z=3;

    }
    else if(nearest_object->model()=="statue_tall_1"){
      std::cout << "chuckles i'm in danger " << nearest_object->model() << std::endl;
      float X=0.60;
      float Y=0.60;
      float Z=2.5;

    }
    else if(nearest_object->model()=="chair_park_1"){
      std::cout << "chuckles i'm in danger " << nearest_object->model() << std::endl;
      float X=2;
      float Y=1;
      float Z=1.2;

    }
    else if(nearest_object->model()=="trashcan_park_1"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=0.60;
      float Y=0.60;
      float Z=0.80;

    }
    else if(nearest_object->model()=="sink"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=1.0;
      float Y=1.0;
      float Z=1.2;

    }
    else if(nearest_object->model()=="burner_stove"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=1.0;
      float Y=1.0;
      float Z=1.2;

    }
    else if(nearest_object->model()=="table_ikea_bjursta"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=1.5;
      float Y=1.5;
      float Z=1.2;

    }
    else if(nearest_object->model()=="chair_ikea_borje"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=0.60;
      float Y=0.60;
      float Z=1.5;

    }
    else if(nearest_object->model()=="couch"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=2.0;
      float Y=1.0;
      float Z=1.0;

    }
    else if(nearest_object->model()=="cabinet_ikea_malm_big"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=1.5;
      float Y=1.0;
      float Z=1.0;

    }
    else if(nearest_object->model()=="table_tv"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=1.2;
      float Y=0.8;
      float Z=0.80;

    }
    else if(nearest_object->model()=="tv_samsung"){
      std::cout << "chuckles, i'm in danger " << nearest_object->model() << std::endl;
      float X=1.0;
      float Y=1.5;
      float Z=1.0;

    }

    else{
      std::cout << "chuckles, it is a car right? " << nearest_object->model() << std::endl;
      X=4.6;
      Y=1.9;
      Z=1.7;
}
    float Dmin_1=(std::max(nearest_object->max().z(),Z)-1)/tan(0.524);
    float Dmin_2=1/tan(0.524);
    float Dmin=std::max(Dmin_1,Dmin_2);
    CLEARANCE=std::min(Dmin,float(2.5));
    std::cout << "So, I am intelligently fixing the distance at " << CLEARANCE << " because Dmin=" << Dmin << std::endl;
  }
  //--------------------------------------------// 
    Isometry3fVector candidate_views;
  squaredDistances[0]=pow(nearest_object->position().x()-(nearest_object->max()[0]+OFFSET),2);
  squaredDistances[1]=pow(nearest_object->position().y()-(nearest_object->max()[1]+OFFSET),2);
  _radius=sqrt(squaredDistances[0]+squaredDistances[1])+CLEARANCE;
  //--------------SECTION 3.2.2-----------------//
  if(true){
    octomap::point3d sensorOrigins(_camera_pose.translation()[0],_camera_pose.translation()[1],_camera_pose.translation()[2]);

    float alpha=atan2((nearest_object->position().y()-sensorOrigins.y()),(nearest_object->position().x()-sensorOrigins.x()));

    squaredDistances[0]=pow(nearest_object->position().y()-sensorOrigins.y(),2);
    squaredDistances[1]=pow(nearest_object->position().x()-sensorOrigins.x(),2);
    float dp=sqrt(squaredDistances[0]+squaredDistances[1]);
  
    squaredDistances[0]=pow(nearest_object->position().y()-nearest_object->max().y(),2);
    squaredDistances[1]=pow(nearest_object->position().x()-nearest_object->max().x(),2);
    float malakies=sqrt(squaredDistances[0]+squaredDistances[1]);
    


    float adjustedRadius=CLEARANCE+(std::max((std::max(X,Y)/2),malakies));
    std::cout << "malakies: " << malakies << " max= " << (std::max(X,Y)/2) << std::endl;

    float dis = adjustedRadius+dp-_radius; 
    std::cout << "dis: " << dis << " adjustedRadius= " << adjustedRadius << std::endl;
    std::cout << "_radius: " << _radius  << std::endl;
    Eigen::Vector2f AproximatedCentroid;
    if (dp>dis){
      AproximatedCentroid.x()=sensorOrigins.x()+(dis*cos(alpha));
      AproximatedCentroid.y()=sensorOrigins.y()+(dis*sin(alpha));
    } else {
      AproximatedCentroid.x()=nearest_object->position().x();
      AproximatedCentroid.y()=nearest_object->position().y();

    }



    std::cout << "NEW CENTROID AT: " << AproximatedCentroid.x() << " - " << AproximatedCentroid.y() << std::endl;
    for(int i=0; i<8; i++){
      float alpha=i*(2*M_PI/((float)8));
      float x=adjustedRadius*cos(alpha);
      float y=adjustedRadius*sin(alpha);
      float theta=atan2(-y,-x);

      Eigen::Isometry3f T = v2t(Eigen::Vector3f(AproximatedCentroid.x()+x,AproximatedCentroid.y()+y,theta));

      candidate_views.push_back(T);
    }
  }
  //--------------------------------------------//

  else{

    for(int i=0; i<8; i++){
      float alpha=i*(2*M_PI/((float)8));
      float x=_radius*cos(alpha);
      float y=_radius*sin(alpha);
      float theta=atan2(-y,-x);

      Eigen::Isometry3f T = v2t(Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,theta));
	std::cout << "NO HAY ERROR NO HAY ERROR, ESPAÑA TE ATACA! ";
      candidate_views.push_back(T);
    }
  }







  return candidate_views;
}

void SemanticExplorer::computeNBV(const Isometry3fVector& candidate_views, const ObjectPtr& nearest_object){
  if(candidate_views.empty())
    throw std::runtime_error("[SemanticExplorer][computeNBV]: no candidate views!");

  //clear queue
  _views = ScoredPoseQueue();

  int unn_max=-1;
  Vector3fPairVector rays;

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
      0.0, 554.25, 240.5,
      0.0,    0.0,   1.0;
  Eigen::Matrix3f inverse_camera_matrix = K.inverse();

  //camera offset
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

  //simulate view
  for(int i=0; i<candidate_views.size(); ++i){

    const Eigen::Isometry3f& T = candidate_views[i];

    //set ray origin to camera pose
    octomap::point3d origin(T.translation().x(),T.translation().y(),T.translation().z());
    std::cerr << "Evaluating view: " << origin << " => ";

    //generate rays
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
    int occ=0,fre=0,unn=0;
    std::vector<octomap::point3d> ray;
    for (int r=0; r<480; r=r+40)
      for (int c=0; c<640; c=c+40){

        //compute ray endpoint
        end=inverse_camera_matrix*Eigen::Vector3f(c,r,1);
        end.normalize();
        end=5*end;
        end=camera_offset*end;
        end=T*end;
        octomap::point3d dir(end.x(),end.y(),end.z());

        //store ray
        rays.push_back(std::make_pair(T.translation(),end));

        //ray casting
        ray.clear();
        if(nearest_object->octree()->computeRay(origin,dir,ray)){
          for(const octomap::point3d voxel : ray){

            if(!nearest_object->inRange(voxel.x(),voxel.y(),voxel.z(),0.0))
              continue;

            octomap::OcTreeNode* n = nearest_object->octree()->search(voxel);
            if(n){
              double value = n->getOccupancy();
              if(value>0.5)
                occ++;
              else
                fre++;
              break;
            } else
              unn++;
          }
        }
      }
    std::cerr << "occ: " << occ << " - unn: " << unn << " - fre: " << fre << std::endl;

    if(unn>unn_max){
      unn_max = unn;
      _rays = rays;
    }
    ScoredPose view;
    view.score = unn;
    view.pose = t2v(T);
    _views.push(view);
    rays.clear();
  }

  //  std::cerr << "Nearest object occ voxels: " << _nearest_object->occVoxelCloud()->size() << std::endl;
  //  std::cerr << "Nearest object fre voxels: " << _nearest_object->freVoxelCloud()->size() << std::endl;
  //  pcl::io::savePCDFileASCII("occ_cloud.pcd", *_nearest_object->occVoxelCloud());
  //  pcl::io::savePCDFileASCII("fre_cloud.pcd", *_nearest_object->freVoxelCloud());
  //  serializeRays(_rays,"rays.txt");
}



/*------------------------NBV_Jose2------------------------*/

std::vector<int> SemanticExplorer::computeNBV_Jose(const Isometry3fVector& candidate_views, ObjectPtr& nearest_object){
  if(candidate_views.empty())
    throw std::runtime_error("[SemanticExplorer][computeNBV_Jose]: no candidate views!");

  //clear queue
  _views = ScoredPoseQueue();
  std::cerr << "[SemanticExplorer][computeNBV_Jose]: OBJECT TIMESTAMP: " << nearest_object->ocupancy_volume() << std::endl;
  octomap::point3d sensorOrigin(_camera_pose.translation()[0],_camera_pose.translation()[1],_camera_pose.translation()[2]);

  //>>>>>>>>>> Get nearest_object info <<<<<<<<<<
  std::cerr<<"camera at "<<sensorOrigin;
  PointCloud::Ptr cloud ;
  Eigen::Vector4f centroid;
  cloud=nearest_object->cloud();
  Eigen::Vector3f& max=nearest_object->max();
  Eigen::Vector3f& min=nearest_object->min();
  pcl::compute3DCentroid(*cloud,centroid);
  std::cerr << "[SemanticExplorer][computeNBV_Jose]: centroid! "<<centroid(0)<<"  "<<centroid(1)<<"  "<<centroid(2)<< std::endl;
  float cloudCentroid[3]={centroid(0),centroid(1),centroid(2)}; //TODO use centroid and delete cloudCentroid
  Eigen::Vector3f squaredDistances;
  float distance;		//  Distance from the sensorOrigin and the new background point
  octomap::point3d Point3dwall(1,0,0);    //  each point3d to be inserted into Pointwall
  octomap::point3d iterator;      //  Helper needed for castRay function

    //>>>>>>>>>> Compute Next Best View candidates <<<<<<<<<<

    /*  The candidates will be computed at a constant distance from the object, all the views
            will be pointing towards the centroid of the cloud. The candidates are computed in Z=0
            by circular tessellation. Further updates will include sphere tessellation for 3D NBV.    */
    std::vector<int> scored_candidate_poses;
    int NBV_CANDIDATENUMBER=8;		//  Number of candidates
    float AngleBetweenCandidate=6.2832/NBV_CANDIDATENUMBER;		//	AngleBetweenCandidate= 360 degrees (2*pi)/ Number of candidates
    float Candidates [NBV_CANDIDATENUMBER][5];		//	Array holding NBV candidates position [x,y,z],roll, and Occlussion Aware VI
    float P_OCCUPATION=0.5;		//	Probability of a random voxel being occupied
    int iNBV=0;     //  Index of the Next Best View in Candidates[i]
    float NBV_DISTANCE=0.4;      //  Closest distance from the NBV candidates to the cloud
    octomap::Pointcloud variablePointwall;      //  variablePointwall hold the FoV of the NBV candidates

    octomap::Pointcloud NBVpointwall;       //  NBVpointwall is a pointcloud holding the Field of View of the NBV candidates

    /*  raytrace from NBVcandidate position to each point in NBVpointwall,
            so this needs to be always behind the cloudAndUnknown voxels    */

    squaredDistances[0]=pow(centroid(0)-(max[0]),2);
    squaredDistances[1]=pow(centroid(1)-(max[1]),2);
    float distance_centroid_furthermostpoint=sqrt(squaredDistances[0]+squaredDistances[1]);
    const Eigen::Isometry3f& SOCLOSE = candidate_views[0];
    squaredDistances[0]=pow(centroid(0)-SOCLOSE.translation().x(),2);
    squaredDistances[1]=pow(centroid(1)-SOCLOSE.translation().x(),2);
    float distance_NBV_Centroid=sqrt(squaredDistances[0]+squaredDistances[1]);
    distance=(distance_centroid_furthermostpoint+distance_NBV_Centroid);
    Point3dwall.x()=distance;


    float yLimit = distance*tan(0.51051);
    float zLimit = distance*tan(0.40666);
    float yPixelDistance = 2*yLimit/320;
    float zPixelDistance = 2*zLimit/240;

    //  Compute NBVpointwall at distance
    for(int ii=1;ii<321;ii++){

        for(int iii=1;iii<241;iii++){

            Point3dwall.y()= (-yLimit)+(ii*yPixelDistance);
            Point3dwall.z()= (-zLimit)+(iii*zPixelDistance);
            NBVpointwall.push_back(Point3dwall);

        }

    }
    std::cerr << "[SemanticExplorer][computeNBV_Jose]: Raytracing... "<< std::endl;
    clock_t tStart = clock();

    for(int i=0;i<candidate_views.size();i++){
        const Eigen::Isometry3f& T = candidate_views[i];
        /* 	The Pointcloud representing the FoV of the kinect is rotated and translated
                so that computeRayKeys can raytrace from the NBVcandidate position until
                each of this Pointcloud points.    */
        
        iterator.x()=Candidates[i][0]=T.translation().x();       // [x]
        iterator.y()=Candidates[i][1]=T.translation().y();       // [y]
        iterator.z()=Candidates[i][2]=T.translation().z();                                          // [z]
        Candidates[i][3]=T.linear().eulerAngles(0, 1, 2)[2];
        Candidates[i][4]=0;		//	Set 0 the candidates Occlussion Aware VI
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		//	Vector with the NBVcandidate coordinates
        octomath::Quaternion Rotation2(0,0,Candidates[i][3]);		//	Quaternion containing the yaw of NBVcandidate
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);		//	Pose6D contains the pose (rotation and translation) of the NBVcandidate
        variablePointwall=NBVpointwall;		//	Reset variablePointwall
        variablePointwall.transform(RotandTrans2);		//	Change the pose of variablePointwall


	

        //  Occlussion Aware VI is used as method to compute "how good" is a candidate
        octomap::KeyRay rayBeam; 		//	Contains the position of each voxel in a ray
        int unknownVoxelsInRay=0;		//	Counter of unknown voxels found in a ray
        
        for(int ii=0;ii<variablePointwall.size();ii++){		//	iterate over all the pointwall
          bool breaker=false;
          //	Get the position of each voxel in a ray starting from NBVcandidate to each point in variablePointwall
          nearest_object->octree()->computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
          //std::cout<< "Starting!";
          for(octomap::OcTreeKey rayKey : rayBeam){
            octomap::OcTreeNode* node = nearest_object->octree()->search(rayKey);
            octomap::point3d keyCoordinate = nearest_object->octree()->keyToCoord(rayKey);

            if(node!=NULL){		//	when !=NULL, it found a free voxel or a known occupied voxel

              if (node->getOccupancy()>0.49 && node->hasChildren()){
              break;
              }
      

            }else if(nearest_object->inRange(keyCoordinate.x(),keyCoordinate.y(),keyCoordinate.z(),0.1)){
              unknownVoxelsInRay++;
              Candidates[i][4]+=pow(P_OCCUPATION,unknownVoxelsInRay);
              breaker=true;
            }else if(breaker){
              break;
            }

          }
            
          unknownVoxelsInRay=0;		//	set to 0 after it stops raytracing
        }

        std::cout<< std::endl<<"from (" << Candidates[i][0] << ") ("<< Candidates[i][1] << ") ("<< Candidates[i][2]<< ") Angle: "<< Candidates[i][3]<<" VI: "<<Candidates[i][4];
        ScoredPose view;
        view.score = (int)Candidates[i][4];
        Eigen::Vector3f NBVpos;
        scored_candidate_poses.push_back(Candidates[i][4]);
        NBVpos[0]=Candidates[i][0];
        NBVpos[1]=Candidates[i][1];
        NBVpos[2]=Candidates[i][3];     // just need 2D
        //NBVpos[3]=Candidates[i][3];
        view.pose = NBVpos;
        _views.push(view);

        //	search the index of the biggest VI
        if(Candidates[i][4]>Candidates[iNBV][4]){

            iNBV=i;

        }

    }
    std::cout<< std::endl<<"Elapsed time: " << (double)(clock() - tStart)/CLOCKS_PER_SEC<< std::endl;
    //	Print NBV
    std::cout<< std::endl<<"NEXT BEST VIEW IS (" << Candidates[iNBV][0] << ") ("<< Candidates[iNBV][1] << ") ("<< Candidates[iNBV][2]<<")"<< std::endl;
  return scored_candidate_poses;
}

/*-----------------------/NBV_Jose2/-----------------------*/

void SemanticExplorer::setProcessed(const ObjectPtr& nearest_object){
  if(!nearest_object)
    throw std::runtime_error("[SemanticExplorer][setProcessed]: no nearest object!");

  StringObjectPtrMap::iterator it = _objects.find(nearest_object->model());
  if(it!=_objects.end()){
    const ObjectPtr& o = it->second;
    _processed.push_back(o->model());
    _objects.erase(it);
  } else
    throw std::runtime_error("[SemanticExplorer][setProcessed]: you're messing up things!");
}

void serializeRays(const Vector3fPairVector& rays, const std::string& filename){
  std::ofstream data;
  data.open(filename);

  for(int i=0; i<rays.size(); ++i){
    const std::pair<Eigen::Vector3f,Eigen::Vector3f>& ray = rays[i];
    const Eigen::Vector3f& first = ray.first;
    const Eigen::Vector3f& second = ray.second;
    data << first.x() << " " << first.y() << " " << first.z() << " ";
    data << second.x() << " " << second.y() << " " << second.z() << std::endl;
  }
  data.close();
}