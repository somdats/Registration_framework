#include"pch.h"
#include"Evaluations_ETH.h"
#include"Common.h"
#include"ErrorMetric.h"
#include<algorithm>
#include <stdexcept>
#include<algorithm>
#include <fstream>
#include <iomanip>


namespace
{
    int indexVisited(std::vector<int> &anVisitedList, int nCurrentIndx)
    {
        int visitedNodes = (int)anVisitedList.size();
        bool flag = false;
        for (int i = 0; i < visitedNodes; i++)
        {
            if (nCurrentIndx == anVisitedList[i])
            {
                flag = true;
                break;
            }
        }

        if (flag)
            return -1;

        else
        {
            anVisitedList.push_back(nCurrentIndx);
            return 0;
        }

    }
    double cTriangleArea(CloudPtr &MeshVertices, pcl::Vertices &cTri)
    {
        double xyDist, yzDist, zxDist;
        double dArea, s;
        int nN1, nN2, nN3;
        Eigen::Vector3f cX, cY, cZ;

        nN1 = cTri.vertices[0];
        nN2 = cTri.vertices[1];
        nN3 = cTri.vertices[2];

        if (nN1 < 0 || nN2 < 0 || nN3 < 0)
            return 0;

        cX = MeshVertices->points[nN1].getVector3fMap();
        cY = MeshVertices->points[nN2].getVector3fMap();
        cZ = MeshVertices->points[nN3].getVector3fMap();

        xyDist = (cX - cY).norm();
        yzDist = (cY - cZ).norm();
        zxDist = (cZ - cX).norm();

        s = (xyDist + yzDist + zxDist) / 2.0;
        dArea = sqrt(s * (s - xyDist) * (s - yzDist) * (s - zxDist));

        return dArea;
    }
    int generatefaceNormals(CloudPtr &MeshVertices, pcl::Vertices & cTri, Eigen::Vector3f  &faceNormals)
    {
        Eigen::Vector3f cBA, cCA, cA, cB, cC, cZdir(0, 0, 1);
        int nN1, nN2, nN3;
        nN1 = cTri.vertices[0];
        nN2 = cTri.vertices[1];
        nN3 = cTri.vertices[2];

        faceNormals.setZero();
        if (nN1 < 0 || nN2 < 0 || nN3 < 0)
            return 0;

        cA = MeshVertices->points[nN1].getVector3fMap();
        cB = MeshVertices->points[nN2].getVector3fMap();
        cC = MeshVertices->points[nN3].getVector3fMap();

        cCA = cC - cA;
        cBA = cB - cA;

        faceNormals = cBA.cross(cCA); //cCA.cross(cBA)
        Eigen::Vector3f  centroid = (cA + cB + cC) / 3.0;
        //double dDir = faceNormals.dot(cZdir); //faceNormals.dot(cZdir);

        //if (dDir < 0.0)
        //{
        //    cCA = cB - cA;
        //    cBA = cC - cA;
        //    faceNormals = cBA.cross(cCA);
        //    
        //}
       /* if (faceNormals.dot((cA - centroid)) < 0.0)
        {
            faceNormals *= -1.0f;
        }
*/
        if (faceNormals.norm() != 0)
            faceNormals.normalize();
        return 1;
    }

    bool IsZeroPoint(Eigen::Vector3f point)
    {
        bool isZero = false;
        Eigen::Vector3f dummypoint(0.0, 0.0, 0.0);
        if (point == dummypoint)
        {
            isZero = true;
            return isZero;
        }
        return isZero;
    }

}
void Evaluations::Reset()
{
    groundTruth_matrices.clear();
    perturbation_matrices.clear();
    Final_registration_matrices.clear();
    comp_files.clear();
    Pose_Type.clear();
    evaluationParameter.clear();
    overlap_ratio.clear();
    times.clear();
}
 void Evaluations::ParseProtocolFile(std::string & fileName)
{
     int k = 1;
     const std::string s = ",";
     //open file
     ifstream in(fileName.c_str());
     if (!in.is_open())
     {
        // print_error("Error loading file\n");
         exit(1);
 
     }
     comp_files.reserve(10000);
     perturbation_matrices.reserve(10000);
     std::string line;
     std::vector<std::string > vec;
     int n_lines = 0;
     while (in.eof() != 1)
     {
         //Always get the line -> so while advances
         getline(in, line);
         // 		cout << line << endl;

         //skip a number k of rows
         n_lines++;
         if (n_lines <= k)
         {
             continue;
         }

         //void lines could be accientaly present, in case continue
         if (line.size() == 0)
         {
             continue;
         }


         //Tokenize the line
         typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
         boost::char_separator<char> sep(s.c_str());
         tokenizer tokens(line, sep);

         //Assign tokens to a string vector
         vec.clear();
         vec.assign(tokens.begin(), tokens.end());



         //just some verbosity
         if (n_lines % 10000 == 0)
         {
             // print_info("line %i\n", n_lines);
         }

    
         //Assign tokens to a string vector
         vec.clear();
         vec.assign(tokens.begin(), tokens.end());
         Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
         temp.row(0) = Eigen::Vector4f(atof(vec[2].c_str()), atof(vec[3].c_str()), atof(vec[4].c_str()), atof(vec[14].c_str()));
         temp.row(1) = Eigen::Vector4f(atof(vec[6].c_str()), atof(vec[7].c_str()), atof(vec[8].c_str()), atof(vec[15].c_str()));
         temp.row(2) = Eigen::Vector4f(atof(vec[10].c_str()), atof(vec[11].c_str()), atof(vec[12].c_str()), atof(vec[16].c_str()));
         temp.col(3) = Eigen::Vector4f(atof(vec[5].c_str()), atof(vec[9].c_str()), atof(vec[13].c_str()), atof(vec[17].c_str()));

         int ref_file_pos = vec[0].find_last_of(".");
         std::string ref_file_name = vec[0];
         ref_file_name.replace(ref_file_pos + 1, vec[0].size(), "ply");

         int read_file_pos = vec[1].find_last_of(".");
         std::string read_file_name = vec[1];
         read_file_name.replace(read_file_pos + 1, vec[1].size(), "ply");
         read_file_name.erase(std::remove_if(read_file_name.begin(), read_file_name.end(), std::isspace), read_file_name.end());
         std::pair<std::string, std::string> name_pair;
         name_pair = std::make_pair(ref_file_name, read_file_name);
         comp_files.push_back(name_pair);
         perturbation_matrices.push_back(temp);
     }
     in.close();
}

 void Evaluations::ParseValidationFile(std::string & fileName)
 {
     int k = 1;
     const std::string s = ",";
     //open file
     ifstream in(fileName.c_str());
     if (!in.is_open())
     {
         // print_error("Error loading file\n");
         exit(1);

     }
     Pose_Type.reserve(10000);
     groundTruth_matrices.reserve(10000);
     overlap_ratio.reserve(10000);
     std::string line;
     std::vector<std::string > vec;
     int n_lines = 0;
     while (in.eof() != 1)
     {
         //Always get the line -> so while advances
         getline(in, line);
         // 		cout << line << endl;

         //skip a number k of rows
         n_lines++;
         if (n_lines <= k)
         {
             continue;
         }

         //void lines could be accientaly present, in case continue
         if (line.size() == 0)
         {
             continue;
         }


         //Tokenize the line
         typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
         boost::char_separator<char> sep(s.c_str());
         tokenizer tokens(line, sep);

         //Assign tokens to a string vector
         vec.clear();
         vec.assign(tokens.begin(), tokens.end());



         //just some verbosity
         if (n_lines % 10000 == 0)
         {
             // print_info("line %i\n", n_lines);
         }


         //Assign tokens to a string vector
         vec.clear();
         vec.assign(tokens.begin(), tokens.end());
         Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
         temp.row(0) = Eigen::Vector4f(atof(vec[2].c_str()), atof(vec[3].c_str()), atof(vec[4].c_str()), atof(vec[14].c_str()));
         temp.row(1) = Eigen::Vector4f(atof(vec[6].c_str()), atof(vec[7].c_str()), atof(vec[8].c_str()), atof(vec[15].c_str()));
         temp.row(2) = Eigen::Vector4f(atof(vec[10].c_str()), atof(vec[11].c_str()), atof(vec[12].c_str()), atof(vec[16].c_str()));
         temp.col(3) = Eigen::Vector4f(atof(vec[5].c_str()), atof(vec[9].c_str()), atof(vec[13].c_str()), atof(vec[17].c_str()));

       
         int read_file_pos = vec[1].length();
         std::string read_file_name = vec[1].substr(1, (read_file_pos - 1));
         Pose_Type.push_back(read_file_name);
         groundTruth_matrices.push_back(temp);
         overlap_ratio.push_back(atof(vec[0].c_str()));
     }
     in.close();
 }

 void Evaluations::WriteEvaluationParameter(std::string &FileName)
 {
     FILE *nFile = fopen(FileName.c_str(), "wb");
    
     if (NULL == nFile)
     {
         return ;
     }
     fprintf(nFile, "Overlap\tPose\tRegistrationTime\tRotationError\tTranslationError\n");
   
     /*else
     {*/
         for (int i = 0; i < Pose_Type.size(); i++)
         {
            // myfile << Pose_Type[i] << "," << times[i] << "," << evaluationParameter[i].first << "," << evaluationParameter[i].second << "\n";
             fprintf(nFile, "%f\t%s\t%f\t%f\t%f\r\n", overlap_ratio[i], Pose_Type[i].c_str(), times[i],
                 evaluationParameter[i].first, evaluationParameter[i].second);
         
         }
    // }
      fclose(nFile);
    // myfile.close();
 }
  double Evaluations::ComputeStandardDeviation(std::vector<std::pair<double, double>>&Data)
 {
     float difference = 0;
     double mean = 0;
     std::vector<std::pair<double, double>>copied_data;
     int size = Data.size()* 0.6;
     copied_data.resize(Data.size() - size);
     std::copy(Data.begin() + size, Data.end(), copied_data.begin());
     std::for_each(std::begin(copied_data), std::end(copied_data), [&](const std::pair<double,double> temp)
     {
         mean += temp.first;
     });
     mean = mean / copied_data.size();

     std::for_each(std::begin(copied_data), std::end(copied_data), [&](const std::pair<double, double> temp)
     {
         difference += (temp.first - mean)* (temp.first - mean);
     });
     float stdDev = sqrt(difference / (copied_data.size() - 1));
     return mean;
 }
  void Evaluations::ComputeCumulativeRecallDataSorted(std::vector<double> &Value_type1, std::vector<double>&Value_type2)
  {
      Value_type1.clear();
      Value_type2.clear();
      std::vector<std::pair<double, double>>evaluationParameter_sort;
      evaluationParameter_sort = evaluationParameter;
      typedef std::pair<double, double>value;
      std::sort(evaluationParameter_sort.begin(), evaluationParameter_sort.end(),
          [&](const value& v1, const value& v2)
      {
          return v1.first < v2.first;
      });
      Value_type1.reserve(evaluationParameter_sort.size());
      std::transform(evaluationParameter_sort.begin(),
          evaluationParameter_sort.end(),
          std::back_inserter(Value_type1),
          [](const std::pair<double, double>& p) { return p.first; });
      std::sort(evaluationParameter_sort.begin(), evaluationParameter_sort.end(),
          [&](const value& v1, const value& v2)
      {
          return v1.second < v2.second;
      });
    
     
      Value_type2.reserve(evaluationParameter_sort.size());
      std::transform(evaluationParameter_sort.begin(),
          evaluationParameter_sort.end(),
          std::back_inserter(Value_type2),
          [](const std::pair<double, double>& p) { return p.second; });

      /*std::for_each(evaluationParameter.begin(),
          evaluationParameter.end(), [&](const value& v)
      {
          Value_type2.push_back(v.second);
         
      });*/
  }
  std::vector<double> Evaluations::ComputeCumulativeRecallGraph(std::vector<double> Value_type1, double spacing, std::vector<double>&comparator)
  {
      comparator.clear();
      comparator.reserve(1000);

      for (int i = 0; i < 1000; i++)
      {
          double val = static_cast<double>(i) *  spacing / static_cast<double>(1000);
          comparator.push_back(val);

      }

      // collect the cumulative recall value using the error value and the comparator
      std::vector<double>recallValue;
      recallValue.reserve(1000);
      for (int i = 0; i < comparator.size(); i++)
      {
          int counterLevel = 0;
          for (int j = 0; j < Value_type1.size(); j++)
          {
              if (Value_type1[j] <= comparator[i])
              {
                  counterLevel = counterLevel + 1;
              }
          }
          recallValue.push_back(static_cast<double>(counterLevel) / Value_type1.size());
      }
      return recallValue;
  }
  std::vector<std::pair<double, double>>Evaluations::ReadEvaluationFile(std::string &FileName, std::string pose_type, float overlap)
  {
      std::string full_file_name =  FileName;
      unsigned noOfLines = pct::readLines(full_file_name);
      int fileRead = 0;
      FILE *pFile;
      pFile = fopen(full_file_name.c_str(), "rb");
      std::vector<std::pair<double, double>>output_data;
      std::vector<double>timing;
      timing.reserve(noOfLines - 1);
      output_data.reserve(noOfLines - 1);
      
      if (NULL == pFile)
      {
          std::cout << "Failed to read data file " << std::endl;
          abort();
      }
      char szParam1[50], szParam2[50], szParam3[50], szParam4[50], szParam5[50];
       
      // fields
      fscanf(pFile, "%s %s %s %s %s", szParam1, szParam2, szParam3, szParam4, szParam5);
      for (int i = 1; i < noOfLines; i++)
      {
          fscanf(pFile, "%s %s %s %s %s\n", szParam1, szParam2, szParam3, szParam4, szParam5);
          std::string test(szParam2);
          if (test.compare(pose_type) == 0 || pose_type == " ")
          {
              if (overlap <= atof(szParam1) || 0.0f == atof(szParam1))
              {
                  output_data.push_back(std::make_pair(atof(szParam4), atof(szParam5)));
                  timing.push_back(atof(szParam2));
              }
          }
          
      }
      times = timing;
      evaluationParameter = output_data;
      fclose(pFile);
      return output_data;
  }
  void Evaluations::ReadMethodComparisonData(std::string & fileName, std::vector<Eigen::Matrix4f> & method_results, std::vector<double> &method_execution_time)
  {
      int k = 1;
      const std::string s = ",";
      //open file
      ifstream in(fileName.c_str());
      if (!in.is_open())
      {
          // print_error("Error loading file\n");
          exit(1);

      }
      method_execution_time.reserve(10000);
      method_results.reserve(10000);
      std::string line;
      std::vector<std::string > vec;
      int n_lines = 0;
      while (in.eof() != 1)
      {
          //Always get the line -> so while advances
          getline(in, line);
          // 		cout << line << endl;

          //skip a number k of rows
          n_lines++;
          if (n_lines <= k)
          {
              continue;
          }

          //void lines could be accientaly present, in case continue
          if (line.size() == 0)
          {
              continue;
          }


          //Tokenize the line
          typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
          boost::char_separator<char> sep(s.c_str());
          tokenizer tokens(line, sep);

          //Assign tokens to a string vector
          vec.clear();
          vec.assign(tokens.begin(), tokens.end());



          //just some verbosity
          if (n_lines % 10000 == 0)
          {
              // print_info("line %i\n", n_lines);
          }


          //Assign tokens to a string vector
          vec.clear();
          vec.assign(tokens.begin(), tokens.end());
          Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
          temp.row(0) = Eigen::Vector4f(atof(vec[1].c_str()), atof(vec[2].c_str()), atof(vec[3].c_str()), atof(vec[13].c_str()));
          temp.row(1) = Eigen::Vector4f(atof(vec[5].c_str()), atof(vec[6].c_str()), atof(vec[7].c_str()), atof(vec[14].c_str()));
          temp.row(2) = Eigen::Vector4f(atof(vec[9].c_str()), atof(vec[10].c_str()), atof(vec[11].c_str()), atof(vec[15].c_str()));
          temp.col(3) = Eigen::Vector4f(atof(vec[4].c_str()), atof(vec[8].c_str()), atof(vec[12].c_str()), atof(vec[16].c_str()));

          method_execution_time.push_back(atof(vec[0].c_str()));
          method_results.push_back(temp);
      }
      in.close();
  }
  std::vector<std::pair< std::string, std::string>>Evaluations::GetComparisonDataName()
  {
      return comp_files;
  }
  std::vector<Eigen::Matrix4f> Evaluations::GetGroundTruthData()
  {
      return groundTruth_matrices;
  }
  std::vector<Eigen::Matrix4f> Evaluations::GetPerturbationData()
  {
      return perturbation_matrices;
  }

  std::vector<std::string>Evaluations::GetPoseType()
  {
      return Pose_Type;
  }
  void Evaluations::setComputationTimeforMethod(std::vector<double> &time)
  {
      times = time;
  }
  void Evaluations::setEvaluationParameter(std::vector<std::pair<double, double>> &evaluationParameter_)
  {
      evaluationParameter = evaluationParameter_;
  }

  void Evaluations:: GeneratePlotDataForMethod(std::vector<std::string>&fileNameLocation, std::string &PoseType, std::vector<std::vector<double>>&rotation_data,
      std::vector<std::vector<double>>&translation_data, std::vector<std::vector<double>>&timing_data, float overlap)
  {
     
      rotation_data.reserve(10);
      timing_data.reserve(10);
      translation_data.reserve(10);
      for each ( std::string file in fileNameLocation)
      {
          std::vector<double>rot_data, trans_data;
          ReadEvaluationFile(file, PoseType, overlap);
          ComputeCumulativeRecallDataSorted(rot_data, trans_data);
          rotation_data.push_back(rot_data);
          translation_data.push_back(trans_data);
          timing_data.push_back(times);
          rot_data.clear();
          trans_data.clear();
      }
  }
  void Evaluations::writeFinalRegistrationMatricesForMethod(std::string &FileNameLocation)
  {
      std::fstream txtfile(FileNameLocation, std::fstream::out);
      txtfile << "T00" << "," << "T01" << "," << "T02" << "," << "T03"
          << "," << "T10" << "," << "T11" << "," << "T12" << "," << "T13"
          << "," << "T20" << "," << "T21" << "," << "T22" << "," << "T23"
          << "," << "T30" << "," << "T31" << "," << "T32" << "," << "T33";
      txtfile << std::endl;
      for each(Eigen::Matrix4f reg_matrix in Final_registration_matrices)
      {

          txtfile << reg_matrix(0, 0) << "," << reg_matrix(0, 1) << "," << reg_matrix(0, 2) << "," << reg_matrix(0, 3)
              << "," << reg_matrix(1, 0) << "," << reg_matrix(1, 1) << "," << reg_matrix(1, 2) << "," << reg_matrix(1, 3)
              << "," << reg_matrix(2, 0) << "," << reg_matrix(2, 1) << "," << reg_matrix(2, 2) << "," << reg_matrix(2, 3)
              << "," << reg_matrix(3, 0) << "," << reg_matrix(3, 1) << "," << reg_matrix(3, 2) << "," << reg_matrix(3, 3);
          txtfile << std::endl;
      }
  }

  void Evaluations::CollectFinalRegistrationMatrices(std::vector<Eigen::Matrix4f> &registration_matrices)
  {
      Final_registration_matrices = registration_matrices;
  }

  std::pair<double, double> Evaluations::ComputeStandardDeviationFromTransformationParamter(std::vector<std::pair<double, double>>&Data)
  {
      double diffRotation = 0.0;
      double diffTrans = 0.0;
      double meanRot = 0.0, meanTrans  = 0.0;
      std::vector<std::pair<double, double>>copied_data;
      int size = Data.size() * 0.9;
      copied_data.resize(size);
      std::copy(Data.begin() + size, Data.end(), copied_data.begin());
      std::for_each(std::begin(copied_data), std::end(copied_data), [&](const std::pair<double, double> temp)
      {
          meanRot += temp.first;
          meanTrans += temp.second;
      });
      meanRot = meanRot / copied_data.size();
      meanTrans = meanTrans / copied_data.size();

      std::for_each(std::begin(copied_data), std::end(copied_data), [&](const std::pair<double, double> temp)
      {
          diffRotation += (temp.first - meanRot)* (temp.first - meanRot);
          diffTrans += (temp.second - meanTrans)* (temp.second - meanTrans);
      });
      double stdDevRot = sqrt(diffRotation / (copied_data.size() - 1));
      double stdDevTrans = sqrt(diffTrans / (copied_data.size() - 1));
      std::pair<double, double> error_pair = std::make_pair(stdDevRot, stdDevTrans);
      return error_pair;
  }

  /////////////////////////////////////////////////////
  CloudWithoutType Evaluation_Ext::TransformView(CloudWithoutType &inputCloud, Eigen::Matrix4f & Gt_Target, Eigen::Matrix4f &R_Transformation)
  {
      Eigen::Matrix4f Combine_Transform = Gt_Target.inverse() * R_Transformation;
      CloudWithoutType transformed_cloud = tool::TransFormationOfCloud(inputCloud, Combine_Transform);
      return transformed_cloud;
 }

  std::vector<std::pair<std::string, std::string>> Evaluation_Ext::ReadViewPairForRegistration(const std::string &ViewList)
  {
      int noOfFiles = pct::readLines(ViewList);

      std::vector<std::pair<std::string, std::string>> view_pair_list;
      int numViewpair = (noOfFiles) * (noOfFiles - 1) / 2;
      view_pair_list.reserve(numViewpair);
      std::vector<std::string>Files = iospace::readFilesfromList(ViewList);
      Views = Files;
      for (int i = 0; i < noOfFiles; i++)
      {
          for (int j = i + 1; j < noOfFiles; j++)
          {
              if (Files[i].compare(Files[j]) != 0)
              {
                  std::pair<std::string, std::string> view_pair(Files[i], Files[j]);
                  view_pair_list.push_back(view_pair);
              }
          }
      }
      return view_pair_list;

  }
  std::vector<std::pair<std::string, Eigen::Matrix4f>> Evaluation_Ext::ReadTransformationmatrixFromFile
  (const std::string &matrixFileName)
  {
      if (Views.size() == 0)
      {
#ifdef LOGDATA
          error_log("Fail to read Evaluation Files");
#endif
          abort();
      }
      std::vector<Eigen::Matrix4f> input_gt_matrices;
      std::vector<std::string> fileName;
      bool fake_name = false;
      if (fake_pairs.size() == 0)
      {
          input_gt_matrices.reserve(Views.size());
          fileName.reserve(Views.size());
      }
      else
      {
          input_gt_matrices.reserve(fake_pairs.size());
          fileName.reserve(fake_pairs.size());
          fake_name = true;
      }
      FILE	*inFile = fopen(matrixFileName.c_str(), "rb");
      if (NULL == inFile)
      {
#ifdef LOGDATA
          error_log("Fail to read TransformationMatrix File");
#endif 
          abort();
      }
      int num_Lines = pct::readLines(matrixFileName);
      char szParam1[50], szParam2[50], szParam3[50], szParam4[50];
      // fields
      fscanf(inFile, "%s %s %s %s", szParam1, szParam2, szParam3, szParam4);
      int j = 1;
      for (int i = j; i < num_Lines; i++)
      {
          if (i == Views.size() + 1 && fake_name == false)
              break;
          if (i == fake_pairs.size() + 1 && fake_name == true)
              break;
          fscanf(inFile, "%s\r\n", szParam1);
          fileName.push_back(szParam1);
          Eigen::Matrix4f  TransformationMatrix = Eigen::Matrix4f::Identity();
          for (int k = j + 1; k < (j + 5); k++)
          {
              fscanf(inFile, " %s %s %s %s\r\n", szParam1, szParam2, szParam3,
                  szParam4);
              Eigen::Vector4f r1(atof(szParam1), atof(szParam2), atof(szParam3), atof(szParam4));
              TransformationMatrix.row(k - j - 1) = r1;
          }
          input_gt_matrices.push_back(TransformationMatrix);
      }
      
      std::vector<std::pair<std::string, Eigen::Matrix4f>>view_vs_matrix;
      int nSize = 0;
      if (fake_pairs.size() == 0)
      {
          view_vs_matrix.reserve(Views.size());
          nSize = Views.size();
      }
      
      else
      {
          view_vs_matrix.reserve(fake_pairs.size());
          nSize = fake_pairs.size();
      }
      for (int itr = 0; itr < nSize; itr++)
      {
          view_vs_matrix.push_back(std::pair<std::string, Eigen::Matrix4f>(fileName[itr], input_gt_matrices[itr]));
      }
      if (NULL != inFile)
      {
          fclose(inFile);
          inFile = NULL;
      }
      return view_vs_matrix;
  }
  double Evaluation_Ext::CalculateRMSEForEvaluationDataSet(std::vector<std::pair<std::string, Eigen::Matrix4f>> & view_vs_transform,
      Eigen::Matrix4f &output_transform, std::string source_view, std::string target_view, CloudWithoutType &source_viewCloud, double rmse_unit  = 1.0)
 
  {
  
     Eigen::Matrix4f gt_source = Eigen::Matrix4f::Identity();
     Eigen::Matrix4f gt_target = Eigen::Matrix4f::Identity();
     std::vector<std::pair<std::string, Eigen::Matrix4f>>::iterator itr = std::find_if(view_vs_transform.begin(), view_vs_transform.end(),
         comp_(source_view));
     if (itr != view_vs_transform.end())
         gt_source = itr->second;

     std::vector<std::pair<std::string, Eigen::Matrix4f>>::iterator it = std::find_if(view_vs_transform.begin(), view_vs_transform.end(),
         comp_(target_view));
     if (it != view_vs_transform.end())
         gt_target = it->second;
     
     CloudWithoutType tranformed_combine_source = TransformView(source_viewCloud, gt_target, gt_source);
    
     CloudWithoutType transformed_source = tool::TransFormationOfCloud(source_viewCloud, output_transform);

     double rmse = static_cast<double>(metric::ComputeRMSE(tranformed_combine_source, transformed_source, rmse_unit));
     return rmse;

  }
  void Evaluation_Ext::WriteViewpair(std::vector<std::pair<std::string, std::string>> & view_pair, std::string &FileName)
  {
      FILE	*g_pLogFile = fopen(FileName.c_str(), "wb");
      if (NULL == g_pLogFile)
      {
          abort();
      }
      fprintf(g_pLogFile, "\tSource\tTarget\n");
      for (int i = 0; i < view_pair.size(); i++)
      {
          fprintf(g_pLogFile, "%s\t%s\r\n", view_pair[i].second.c_str(), view_pair[i].first.c_str());
      }
      if (NULL != g_pLogFile)
      {
          fclose(g_pLogFile);
          g_pLogFile = NULL;
      }
  }
  void Evaluation_Ext::CompareRMSEWithThreshold(const std::string fileName, double Threshold, int &trueCount, int &falseCount)
  {
      // Initialize count
      trueCount = 0;
      falseCount = 0;
      // read rmse values from a file
      int numRMSE = pct::readLines(fileName);
      std::vector<std::string>rmse_value = iospace::readFilesfromList(fileName);

      for (int i = 0; i< rmse_value.size(); i++)
      {
          std::string value = rmse_value[i];
          double val = static_cast<double>(atof(value.c_str()));
          if (val < 5.0 ) // Threshold
          {
              trueCount++;
          }
          else
          {
              std::cout << " false case:" << i << std::endl;
          }
      }
      falseCount = numRMSE - trueCount;
  }
  double Evaluation_Ext::ComputeMeshResolutionForAllViews(const std::string &dirName, const std::string &fileExt, const std::string &absMeshResFile)
  {
      double MeshRes = 0.0;
      bool fileExists = iospace ::ExistsFile(absMeshResFile);

      if (fileExists && (absMeshResFile != ""))
      {
         iospace::LoadSingleValue<double>(absMeshResFile, MeshRes);
          return MeshRes;
      }
      std::vector<std::string> AllFiles;// = iospace::readFilesfromList(fileNameWithPath);
      iospace::FindFilesEndWith(dirName, fileExt, AllFiles);
      int nSize = AllFiles.size();
      
      for  each (std::string file in AllFiles)
      {
         
          pcl::PolygonMesh mesh;
          pcl::io::loadPLYFile(file, mesh);
          double iMeshRes = ComputeMeshResolution(mesh);
          MeshRes += iMeshRes;
      }
      MeshRes = MeshRes / static_cast<double>(nSize);
      if (absMeshResFile != "")
      {
        iospace::  SaveSingleValue<double>(absMeshResFile, MeshRes);
      }
      return MeshRes;
  }
  double Evaluation_Ext::ComputeMeshResolution(pcl::PolygonMesh &mesh)
  {
      CloudWithColorPtr cloud1(new pcl::PointCloud<PointColorType>());
      pcl::fromPCLPointCloud2(mesh.cloud, *cloud1);
      std::vector<pcl::Vertices> polygons;
      std::vector<double>arEdgeLength;
      arEdgeLength.reserve(3 * mesh.polygons.size());
      for (std::vector<pcl::Vertices>::iterator it = mesh.polygons.begin(); it != mesh.polygons.end(); ++it)
      {
          Eigen::Vector3f v1 (0.0,0.0,0.0), v2(0.0, 0.0, 0.0), v3(0.0, 0.0, 0.0);
          v1 = cloud1->points[it->vertices[0]].getVector3fMap();
          v2 = cloud1->points[it->vertices[1]].getVector3fMap();
          v3 = cloud1->points[it->vertices[2]].getVector3fMap();
          arEdgeLength.push_back((v1 - v2).norm());
          arEdgeLength.push_back((v2 - v3).norm());
          arEdgeLength.push_back((v3 - v1).norm());

      }

      // compute average edge length
    double  rAvgEdgeLength = 0.0;
      for ( int iEdgeCount = 0; iEdgeCount < arEdgeLength.size(); iEdgeCount++)
      {
          rAvgEdgeLength += arEdgeLength[iEdgeCount];
      }
      rAvgEdgeLength /= (double)arEdgeLength.size();
      return rAvgEdgeLength;
  }
  std::vector<std::pair<std::string, std::string>>Evaluation_Ext::GenerateViewPairForEvaluation(const std::string &dirName, const std::string &fileExt)
  {
      std::vector<std::string>fileNames;
      iospace::FindFilesEndWith(dirName, fileExt, fileNames,true);
      Views = fileNames;
      std::vector<std::pair<std::string, std::string>> view_pair_list;
      int numViewpair = (fileNames.size()) * (fileNames.size() - 1) / 2;
      view_pair_list.reserve(numViewpair);
      fake_pairs.reserve(numViewpair);
      if (iospace::GetRelativeName(dirName).find("Room") != std::string::npos)
      {
          int pa = 0;
          for (int i = 0; i < fileNames.size(); i++)
          {
              for (int j = i + 1; j < fileNames.size(); j++)
              {
                  if (fileNames[i].compare(fileNames[j]) != 0)
                  {
                      std::pair<std::string, std::string> view_pair(fileNames[i], fileNames[j]);
                      view_pair_list.push_back(view_pair);
                      std::stringstream strste;
                      strste << pa << "_" << i << "_" << j<< ".ply";
                      std::pair<std::string, std::string> fake_pair(strste.str(), fileNames[j]);
                      fake_pairs.push_back(fake_pair);
                      pa++;
                  }
               
              }
          }
          return view_pair_list;
      }
      for (int i = 0; i < fileNames.size(); i++)
      {
          for (int j = i + 1; j < fileNames.size(); j++)
          {
              if (fileNames[i].compare(fileNames[j]) != 0)
              {
                  std::pair<std::string, std::string> view_pair(fileNames[i], fileNames[j]);
                  view_pair_list.push_back(view_pair);
              }
          }
      }

      return view_pair_list;
  }
  void Evaluation_Ext::GenerateFaceNeighborhoodFromMesh(pcl::PolygonMesh &mesh, pointNeigbor_  *ptNeighbors)
  {
      int numFaces = mesh.polygons.size();
      std::vector<pcl::Vertices>Tri;
      int faceCount = 0;
#pragma omp parallel for
      for (int it = 0; it<numFaces; ++it)
      {
          pcl::Vertices Tri = mesh.polygons[it];
          if (Tri.vertices[0] < 0 && Tri.vertices[1] < 0 && Tri.vertices[2] < 0)
              continue;
          else
          {
              ptNeighbors[Tri.vertices[0]].m_anFaceIndicesList.push_back(it);
              ptNeighbors[Tri.vertices[1]].m_anFaceIndicesList.push_back(it);
              ptNeighbors[Tri.vertices[2]].m_anFaceIndicesList.push_back(it);

              ptNeighbors[Tri.vertices[0]].m_anVertexIndxList.push_back(Tri.vertices[1]);
              ptNeighbors[Tri.vertices[0]].m_anVertexIndxList.push_back(Tri.vertices[2]);

              ptNeighbors[Tri.vertices[1]].m_anVertexIndxList.push_back(Tri.vertices[0]);
              ptNeighbors[Tri.vertices[1]].m_anVertexIndxList.push_back(Tri.vertices[2]);

              ptNeighbors[Tri.vertices[2]].m_anVertexIndxList.push_back(Tri.vertices[0]);
              ptNeighbors[Tri.vertices[2]].m_anVertexIndxList.push_back(Tri.vertices[1]);
              faceCount++;
              
          }
      }
  }
  void Evaluation_Ext::ComputePointNormal(pcl::PolygonMesh &mesh, std::vector<Eigen::Vector3f> &Normals,
      pointNeigbor_  *ptNeighbors)
  {
      
     
      double dSumArea;
      int nFlagStat;
      CloudPtr cloud1(new pcl::PointCloud<PointType>());
      pcl::fromPCLPointCloud2(mesh.cloud, *cloud1);
      Normals.resize(cloud1->points.size());
      
#ifdef _WIN32
#pragma omp parallel for
#else
#pragma omp parallel for collapse(2)
#endif
      for (int pointItr = 0; pointItr < cloud1->points.size(); pointItr++)
      {
          
          int numNeighbor = ptNeighbors[pointItr].m_anFaceIndicesList.size();
          Eigen::Vector3f cTemp(0, 0, 0);
          dSumArea = 0.0;
          std::vector<int> anVisitedList;
          anVisitedList.reserve(numNeighbor);
          pointNeigbor_ ptNeigborhood = ptNeighbors[pointItr];
          for (int faceIndexItr = 0; faceIndexItr < numNeighbor; faceIndexItr++)
          {
              int anIndx = ptNeigborhood.m_anFaceIndicesList[faceIndexItr];
              pcl::Vertices Tri = mesh.polygons[anIndx];
              if (Tri.vertices[0] < 0 && Tri.vertices[1] < 0 && Tri.vertices[2] < 0)
                  continue;
              nFlagStat = indexVisited(anVisitedList, anIndx);
              if (nFlagStat == -1)
                  continue;
              Eigen::Vector3f faceNormal;
              generatefaceNormals(cloud1, Tri, faceNormal);
              double dArea = cTriangleArea(cloud1, Tri);
              cTemp = cTemp + dArea * faceNormal;
              dSumArea += dArea;

          }
          if (dSumArea != 0)
              cTemp = cTemp / dSumArea;

          if (cTemp.squaredNorm() != 0)
              cTemp.normalize();

          Normals[pointItr] = cTemp;
        
      }
  }
  void Evaluation_Ext::GenerateFaceNeighborList(pcl::PolygonMesh &mesh, std::vector<std::vector<int>> &Facelist, pointNeigbor_  *ptNeighbors)
  {
      size_t numFace = mesh.polygons.size();
      Facelist.resize(numFace);
      for (size_t faceItr = 0; faceItr < numFace; faceItr++)
      {
          pcl::Vertices Tri = mesh.polygons[faceItr];

          if (Tri.vertices[0] < 0 && Tri.vertices[1] < 0 && Tri.vertices[2] < 0)
              continue;
          for (int ptNeighItr = 0; ptNeighItr < ptNeighbors[Tri.vertices[0]].m_anFaceIndicesList.size(); ptNeighItr++)
          {
              if (faceItr == ptNeighbors[Tri.vertices[0]].m_anFaceIndicesList[ptNeighItr])
                  continue;
              Facelist[faceItr].push_back(ptNeighbors[Tri.vertices[0]].m_anFaceIndicesList[ptNeighItr]);
          }

          for (int ptNeighItr = 0; ptNeighItr < ptNeighbors[Tri.vertices[1]].m_anFaceIndicesList.size(); ptNeighItr++)
          {
              if (faceItr == ptNeighbors[Tri.vertices[1]].m_anFaceIndicesList[ptNeighItr])
                  continue;
              Facelist[faceItr].push_back(ptNeighbors[Tri.vertices[1]].m_anFaceIndicesList[ptNeighItr]);
          }


          for (int ptNeighItr = 0; ptNeighItr < ptNeighbors[Tri.vertices[2]].m_anFaceIndicesList.size(); ptNeighItr++)
          {
              if (faceItr == ptNeighbors[Tri.vertices[2]].m_anFaceIndicesList[ptNeighItr])
                  continue;
              Facelist[faceItr].push_back(ptNeighbors[Tri.vertices[2]].m_anFaceIndicesList[ptNeighItr]);
          }

      }
  }
  void Evaluation_Ext::ComputePointNormal(const std::string &dirName, const std::string &fileExt, const std::string tarDir)
  {
      std::vector<std::string> AllFiles;// = iospace::readFilesfromList(fileNameWithPath);
      iospace::FindFilesEndWith(dirName, fileExt, AllFiles,true);
      Eigen::Vector3f dummyNormal(0.0, 0.0, 0.0);
      for  each (std::string file in AllFiles)
      {
          std::vector<Eigen::Vector3f> normals;
          pcl::PolygonMesh mesh;
          std::string finalPath = dirName + "/" + file;
          pcl::io::loadPLYFile(finalPath, mesh);
          std::string normal = "normal_x";
         
          auto it = std::find_if(std::begin(mesh.cloud.fields), std::end(mesh.cloud.fields), [=](pcl::PCLPointField const& f) {
              return (f.name == normal);
          });
          if (it == std::end(mesh.cloud.fields))
          {
              CloudPtr cloud1(new pcl::PointCloud<PointType>());
              pcl::fromPCLPointCloud2(mesh.cloud, *cloud1);

              SPointNeighbor *pcNeigborhood = new SPointNeighbor[cloud1->points.size()];
              GenerateFaceNeighborhoodFromMesh(mesh, pcNeigborhood);
              std::vector<std::vector<int>> Facelist;
              // GenerateFaceNeighborList(mesh, Facelist, pcNeigborhood);
              ComputePointNormal(mesh, normals, pcNeigborhood);

              // takes care if the normals are zero
              std::vector<PointType>refined_output_points;
              std::vector<Eigen::Vector3f>refined_normals;
              refined_output_points.reserve(cloud1->points.size());
              refined_normals.reserve(cloud1->points.size());
              for (size_t normItr = 0; normItr < normals.size(); normItr++)
              {
                  Eigen::Vector3f norm = normals[normItr];
                  if (IsZeroPoint(norm) == false)
                  {
                      refined_output_points.push_back(cloud1->points[normItr]);
                      refined_normals.push_back(norm);
                  }
              }
              cloud1->points.assign(refined_output_points.begin(), refined_output_points.end());
              cloud1->width = static_cast<uint32_t>(refined_output_points.size());
              cloud1->height = 1;
              NormalPtr PointNormals(new pcl::PointCloud<NormalType>);
              PointNormals->width = cloud1->width;
              PointNormals->height = cloud1->height;
              PointNormals->points.resize(PointNormals->width * PointNormals->height);

              int i;
#pragma omp parallel for
              for (i = 0; i < refined_normals.size(); i++)
              {
                  PointNormals->points[i].getNormalVector3fMap() = refined_normals[i];
              }

              CloudWithNormalPtr  resultant_cloud(new pcl::PointCloud<PointNormalType>);
              pcl::concatenateFields(*cloud1, *PointNormals, *resultant_cloud);
              std::string finalFileName = tarDir + "/" + file;
              iospace::CreateFullPath(iospace::GetPathDir(finalFileName));
              pcl::io::savePLYFile(finalFileName, *resultant_cloud);
              pcl::toPCLPointCloud2(*resultant_cloud, mesh.cloud);
          }
          else
          {
              std::string finalFileName = tarDir + file;
              iospace::CreateFullPath(iospace::GetPathDir(finalFileName));
              pcl::io::savePLYFile(finalFileName, mesh.cloud);
          }
          
      }
  }
  void Evaluation_Ext::PrintResults(const std::string outputFileName, const double Standard_dev, const double MeshResolution, const int totalViews, int trueCount)
  {
      std::ofstream	outFile(outputFileName, std::ios_base::out);
      std::locale defaultLocale("");
      outFile.imbue(defaultLocale);
      if (!outFile.is_open())
      {
          throw  std::runtime_error("ERROR: Impossible to open file " + outputFileName);
      }
      outFile << "filePath:" << outputFileName << "\n";
      outFile << " Standard_deviation:" << Standard_dev << "\n";
      outFile << "MeshResolution:" << MeshResolution << "\n";
      outFile << "TotalViews:" << totalViews << "\n";
      outFile << "TrueCount:" << trueCount << "\n";
      outFile << "SuccessRate:" << (double)trueCount / (double)totalViews << "\n";
  }
  std::vector<std::pair<std::string, std::string>> Evaluation_Ext::GetFakeNamePairs()
  {
      return fake_pairs;
  }

  ///////////////NB: Only for room dataset evaluation////////////////////////
  double Evaluation_Ext::CalculateRMSEForEvaluationDataSet(std::vector<std::pair<std::string, Eigen::Matrix4f>> & view_vs_transform,
      Eigen::Matrix4f &output_transform, std::string fake_view, CloudWithoutType &source_viewCloud, double rmse_unit)
  {
      Eigen::Matrix4f gt_source = Eigen::Matrix4f::Identity();
     
      std::vector<std::pair<std::string, Eigen::Matrix4f>>::iterator itr = std::find_if(view_vs_transform.begin(), view_vs_transform.end(),
          comp_(fake_view));
      if (itr != view_vs_transform.end())
          gt_source = itr->second;

    
      CloudWithoutType tranformed_combine_source = tool::TransFormationOfCloud(source_viewCloud, gt_source);
      CloudWithoutType transformed_source = tool::TransFormationOfCloud(source_viewCloud, output_transform);

      double rmse = static_cast<double>(metric::ComputeRMSE(tranformed_combine_source, transformed_source, rmse_unit));
      return rmse;
      
  }
  void Evaluation_Ext::ComputePointNormal(pcl::PolygonMesh &Mesh )
  {

      std::vector<Eigen::Vector3f> normals;

      std::string normal = "normal_x";

      auto it = std::find_if(std::begin(Mesh.cloud.fields), std::end(Mesh.cloud.fields), [=](pcl::PCLPointField const& f) {
          return (f.name == normal);
      });
      if (it == std::end(Mesh.cloud.fields))
      {
          CloudPtr cloud1(new pcl::PointCloud<PointType>());
          pcl::fromPCLPointCloud2(Mesh.cloud, *cloud1);

          SPointNeighbor *pcNeigborhood = new SPointNeighbor[cloud1->points.size()];
          GenerateFaceNeighborhoodFromMesh(Mesh, pcNeigborhood);
          std::vector<std::vector<int>> Facelist;
          // GenerateFaceNeighborList(mesh, Facelist, pcNeigborhood);
          ComputePointNormal(Mesh, normals, pcNeigborhood);

          // takes care if the normals are zero
          std::vector<PointType>refined_output_points;
          std::vector<Eigen::Vector3f>refined_normals;
          refined_output_points.reserve(cloud1->points.size());
          refined_normals.reserve(cloud1->points.size());
          for (size_t normItr = 0; normItr < normals.size(); normItr++)
          {
              Eigen::Vector3f norm = normals[normItr];
              if (IsZeroPoint(norm) == false)
              {
                  refined_output_points.push_back(cloud1->points[normItr]);
                  refined_normals.push_back(norm);
              }
          }
          cloud1->points.assign(refined_output_points.begin(), refined_output_points.end());
          cloud1->width = static_cast<uint32_t>(refined_output_points.size());
          cloud1->height = 1;
          NormalPtr PointNormals(new pcl::PointCloud<NormalType>);
          PointNormals->width = cloud1->width;
          PointNormals->height = cloud1->height;
          PointNormals->points.resize(PointNormals->width * PointNormals->height);

          int i;
#pragma omp parallel for
          for (i = 0; i < refined_normals.size(); i++)
          {
              PointNormals->points[i].getNormalVector3fMap() = refined_normals[i];
          }

          CloudWithNormalPtr  resultant_cloud(new pcl::PointCloud<PointNormalType>);
          pcl::concatenateFields(*cloud1, *PointNormals, *resultant_cloud);
         
          pcl::toPCLPointCloud2(*resultant_cloud, Mesh.cloud);
      }
     
  }