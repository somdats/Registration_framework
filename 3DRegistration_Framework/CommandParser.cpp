#include"pch.h"
#include"CommandParser.h"

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#define NOMINMAX
#include "windows.h"
#endif

void parser::showHelp(char *fileName)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "Usage: " << fileName << " source_filename.ply target_filename.ply target_pctfile.pct" << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    // TODO include further option
 }

void parser::ParseCommandLine(int argc, char *argv[])
{
    //Show help
    if (pcl::console::find_switch(argc, argv, "-h"))
    {
        showHelp(argv[0]);
        exit(0);
    }
    //Model & scene filenames
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    if (filenames.size() != 2)
    {
        std::cout << "Filenames missing.\n";
        showHelp(argv[0]);
        exit(-1);
    }
    std::vector<int> pctfilenames;
    pctfilenames = pcl::console::parse_file_extension_argument(argc, argv, ".pct");
    if (pctfilenames.size() != 1)
    {
        std::cout << " pct Filenames missing.\n";
        showHelp(argv[0]);
        exit(-1);
    }
    std::vector<int> tfs_matrix;
    tfs_matrix = pcl::console::parse_file_extension_argument(argc, argv, "txt");
    if (tfs_matrix.size() != 1)
    {
        std::cout << " outputtransformationMatrix Filenames missing.\n";
        showHelp(argv[0]);
        exit(-1);
    }
    source_filename_ = argv[filenames[0]];
    target_filename_ = argv[filenames[1]];
    target_pctfile_ = argv[pctfilenames[0]];
    output_transformation_matrix = argv[tfs_matrix[0]];
}

int parser::BatchProcess(int argc, char *argv[], std::vector<std::string> plyFiles, std::vector<std::string> pctFiles)
{
    std::string  input_dir;
    if (pcl::console::parse_argument(argc, argv, "-input_dir", input_dir) != -1)
    {
        PCL_INFO("Input directory given as %s. Batch process mode on.\n", input_dir.c_str());
        if (pcl::console::parse_argument(argc, argv, "-output_dir", output_dir) == -1)
        {
            PCL_ERROR("Need an output directory! Please use -output_dir to continue.\n");
            return (-1);
        }

        // Both input dir and output dir given, switch into batch processing mode
        batch_mode = true;
    }
    if (!batch_mode)
    {
        ParseCommandLine(argc, argv);
    }
    else
    {
        if (input_dir != "" && boost::filesystem::exists(input_dir))
        {

            boost::filesystem::directory_iterator end_itr;
            for (boost::filesystem::directory_iterator itr(input_dir); itr != end_itr; ++itr)
            {
                //  add ply files
                if (!is_directory(itr->status()) && boost::filesystem::extension(itr->path()) == ".ply")
                {
                    plyFiles.push_back(itr->path().string());
                    PCL_INFO("[Batch processing mode] Added %s for processing.\n", itr->path().string().c_str());
                }
                //  add pct files
                if (!is_directory(itr->status()) && boost::filesystem::extension(itr->path()) == ".pct")
                {
                    pctFiles.push_back(itr->path().string());
                    PCL_INFO("[Batch processing mode] Added %s for processing.\n", itr->path().string().c_str());
                }
            }
            // batchProcess(pcd_files, output_dir, resolution);
        }
        else
        {
            PCL_ERROR("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str());
            return (-1);
        }
    }
    return (0);
}
