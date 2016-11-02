#include <itomp_fetch/fetch/fetch.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    
    srand(time(NULL));
    
    ros::init(argc, argv, "itomp_fetch");
    
    itomp_fetch::Fetch fetch;

    // initialize with highest torso position
    fetch.moveTorso(0.35);
    fetch.moveTorso(0.1);
    fetch.moveTorso(0.35);
    fetch.moveTorso(0.1);

    return 0;
}
