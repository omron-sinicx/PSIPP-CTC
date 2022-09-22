FROM ubuntu:20.04

# build tools
RUN apt-get -y update
RUN apt-get install -y \
    wget \
    libssl-dev \
    git

# instal python3
RUN apt-get install -y \
    python3=3.8.2-0ubuntu2 \
    python3-pip=20.0.2-5ubuntu1.6

# install python packages
RUN python3 -m pip install \
    matplotlib==3.5.0 \
    numpy==1.21.4

# install cmake
RUN wget https://github.com/Kitware/CMake/archive/refs/tags/v3.20.2.tar.gz -O cmake.tar.gz
RUN tar zxvf cmake.tar.gz
RUN cd CMake-3.20.2 && ./bootstrap && make -j4 && make install && cd ..
RUN hash -r
RUN rm -rf CMake-3.20.2

# install yamp-cpp
RUN git clone https://github.com/jbeder/yaml-cpp.git
RUN cd yaml-cpp && mkdir build && cd build && cmake .. && make install && cd ..
RUN rm -rf yaml-cpp

# install CGAL
RUN apt-get install -y libcgal-dev=5.0.2-3

# clear cache
RUN rm -rf /var/lib/apt/lists/*

# build PSIPP-CTC
COPY src/ src/
COPY third_party/ third_party/
COPY CMakeLists.txt .
COPY problem_instances/ problem_instances/
COPY tools/ tools/
RUN mkdir build && cd build && cmake .. && make && cd ..