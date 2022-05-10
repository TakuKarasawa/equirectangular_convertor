#include "equirectangular_convertor/equirectangular_convertor.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"equirectangular_convertor");
    EquirectangularConvertor equirectangular_convertor;
    equirectangular_convertor.process();
    return 0;
}
