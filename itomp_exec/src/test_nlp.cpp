#include <itomp_exec/planner/itomp_planner.h>

#include <itomp_exec/nlp/corenlp_parser.h>

#include <ros/ros.h>


void trimCR(char* string)
{
    int n = strlen(string);

    while (n>=0 && string[n] == 13)
        string[n] = 0;
}

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    
    srand(time(NULL));

    ros::init(argc, argv, "test_nlp");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ROS_INFO("timer started");

    while (ros::ok())
    {
        bool string_found = false;
        std::string xml_string;
        char buffer[1024];
        while (gets(buffer))
        {
            string_found = true;

            trimCR(buffer);

            // append to a string, ignoring xml header tags
            if (strncmp(buffer, "<?xml", 5) != 0)
                xml_string += std::string(buffer) + '\n';

            if (strncmp(buffer, "</root>", 7) == 0)
            {
                ROS_INFO("end");
                break;
            }
        }

        if (!string_found)
            break;

        itomp_exec::CoreNLPParser parser(xml_string);
        itomp_exec::LanguageModel* language_model = parser.getLanguageModel();

        ROS_INFO("Time: %lfs", (ros::Time::now() - start_time).toSec());
    }

    ros::shutdown();
    return 0;
}
