bool MainWindow::TestRadiusOutlierRemovalSub(std::string &path_file)
{
    bool ret = false;

    if (!path_file.empty()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile(path_file, *cloud) != -1) {
            ret = NaNRemovalFilter(cloud, cloud_in);
        }
        if (ret) {
            bool negative = false;
            double radius_search = 0.05;
            int min_neighbors = 30;

            QString msg;
            msg.sprintf("[radius = %.3f, neighbors = %d]\n", radius_search, min_neighbors);
            OutputDebugString(msg.toStdWString().data());

            std::string name_process;
            name_process = "filter_1:";
            unsigned int size_in_1 = cloud_in->points.size();
            StartProcessTimer();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_1(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter_1;
            filter_1.setInputCloud(cloud_in);
            filter_1.setRadiusSearch(radius_search);
            filter_1.setMinNeighborsInRadius(min_neighbors);
            filter_1.setNegative(negative);
            filter_1.filter(*cloud_out_1);

            StopProcessTimer(name_process);

            unsigned int size_out_1 = cloud_out_1->points.size();
            msg.sprintf("filter_1:%d -> %d[pts]\n", size_in_1, size_out_1);
            OutputDebugString(msg.toStdWString().data());

            name_process = "filter_2:";
            unsigned int size_in_2 = cloud_in->points.size();
            StartProcessTimer();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_2(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::RadiusOutlierRemoval2<pcl::PointXYZRGB> filter_2;
            filter_2.setInputCloud(cloud_in);
            filter_2.setRadiusSearch(radius_search);
            filter_2.setMinNeighborsInRadius(min_neighbors);
            filter_2.setNegative(negative);
            filter_2.filter(*cloud_out_2);

            StopProcessTimer(name_process);

            unsigned int size_out_2 = cloud_out_2->points.size();
            msg.sprintf("filter_2:%d -> %d[pts]\n", size_in_2, size_out_2);
            OutputDebugString(msg.toStdWString().data());

            bool chk = true;
            if (size_out_1 == size_out_2) {
                for (unsigned int i = 0; i < size_out_1; i++) {
                    if (cloud_out_1->points[i].x != cloud_out_2->points[i].x ||
                            cloud_out_1->points[i].y != cloud_out_2->points[i].y ||
                            cloud_out_1->points[i].z != cloud_out_2->points[i].z) {
                        chk = false;
                        msg = tr("Cloud data different!\n");
                        OutputDebugString(msg.toStdWString().data());
                        break;
                    }
                }
            } else {
                chk = false;
                msg = tr("Cloud size different!\n");
                OutputDebugString(msg.toStdWString().data());
            }
            if (chk) {
                msg = tr("Cloud data same!\n");
                OutputDebugString(msg.toStdWString().data());
            }
        }
    }

    return ret;
}

bool MainWindow::NaNRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    bool ret = false;

    if (cloud_in) {
        ret = true;
        cloud_out.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> index;
        pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*cloud_in, *cloud_out, index);
    }

    return ret;
}

void MainWindow::StartProcessTimer()
{
    // m_TimerProcess is defined by boost::timer::cpu_timer class in MainWindow.
    m_TimerProcess.start();
}

void MainWindow::SplitProcessTimer(std::string &name_process)
{
    boost::timer::cpu_times elapsed = m_TimerProcess.elapsed();
    double time_sec = elapsed.wall * 0.000000001;

    QString msg = tr(name_process.data());
    QString str;
    str.sprintf("%.3f[sec]\n", time_sec);
    msg += str;
    OutputDebugString(msg.toStdWString().data());
}
