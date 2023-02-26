##################
Build instructions
##################

cd ~/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/programming/zmqRemoteApi/clients/cpp/build/
export COPPELIASIM_ROOT_DIR=/home/khai/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04
cmake -DGENERATE=OFF ..
cmake --build . --config Release
