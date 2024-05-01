
#include "metrics.h"

// Function to calculate and return RRE (degrees).
auto compute_rre(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation) -> double
{
    // Extract rotation matrices
    Eigen::Matrix3d R_current = current_transformation.block<3, 3>(0, 0);
    Eigen::Matrix3d R_ground_truth = ground_truth_transformation.block<3, 3>(0, 0);

    // Calculate RRE as the angle between the two rotation matrices
    Eigen::Quaterniond q_current(R_current), q_ground_truth(R_ground_truth);
    double RRE = q_current.angularDistance(q_ground_truth) * (180.0 / M_PI);

    return RRE;
}

// Function to calculate and return RTE (meters).
auto compute_rte(const Eigen::Matrix4d &current_transformation, const Eigen::Matrix4d &ground_truth_transformation) -> double
{
    // Extract translation vectors
    Eigen::Vector3d t_current = current_transformation.block<3, 1>(0, 3);
    Eigen::Vector3d t_ground_truth = ground_truth_transformation.block<3, 1>(0, 3);

    // Calculate RTE as Euclidean distance between the two translation vectors
    double RTE = (t_current - t_ground_truth).norm();
    return RTE;
}

void print_metrics(Pipeline &pipeline, const Dataset &dataset)
{
    const double SUCCESS_RTE = 2.0;
    const int DS_SIZE = 300;
    double easy_total_rte = 0.0;
    double hard_total_rte = 0.0;
    double easy_total_rre = 0.0;
    double hard_total_rre = 0.0;
    int easy_total = 0;
    int hard_total = 0;
    int easy_success = 0;
    int hard_success = 0;
    double easy_time = 0.0;
    double hard_time = 0.0;

    for (int i = 0; i < DS_SIZE; i++)
    {
        // Run current frame through pipeline and time completion
        pcl::console::TicToc time;
        auto const c_pose_est = pipeline.guess_v_pose(dataset.frames[i], dataset.i_pose);
        auto const elapsed = time.toc();

        double rre = compute_rre(c_pose_est, dataset.frames[i].pose_c);
        double rte = compute_rte(c_pose_est, dataset.frames[i].pose_c);
        if (std::find(easy_idxs.begin(), easy_idxs.end(), i) != easy_idxs.end())
        {
            easy_total_rte += rte;
            easy_total_rre += rre;
            if (rte <= SUCCESS_RTE)
            {
                easy_success += 1;
            }
            easy_time += elapsed;
            easy_total += 1;
        }
        else
        {
            hard_total_rte += rte;
            hard_total_rre += rre;
            if (rte <= SUCCESS_RTE)
            {
                hard_success += 1;
            }
            hard_time += elapsed;
            hard_total += 1;
        }
    }

    std::cout << "Avg. RTE (Easy): " << (float)easy_total_rte / (float)easy_total << std::endl;
    std::cout << "Avg. RRE (Easy): " << (float)easy_total_rre / (float)easy_total << std::endl;
    std::cout << "Success Rate (Easy): " << (float)easy_success / (float)easy_total << std::endl;
    std::cout << "Avg. Time (Easy): " << (float)easy_time / (float)easy_total << std::endl;
    std::cout << "Avg. RTE (Hard): " << (float)hard_total_rte / (float)hard_total << std::endl;
    std::cout << "Avg. RRE (Hard): " << (float)hard_total_rre / (float)hard_total << std::endl;
    std::cout << "Success Rate (Hard): " << (float)hard_success / (float)hard_total << std::endl;
    std::cout << "Avg. Time (Hard): " << (float)hard_time / (float)hard_total << std::endl;
}
