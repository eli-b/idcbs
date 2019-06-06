FROM gcc

# This also creates the dir
WORKDIR /libs
RUN git clone https://github.com/google/glog.git
RUN cd glog && ./autogen.sh && ./configure && make && make install
RUN cd /libs; wget https://dl.bintray.com/boostorg/release/1.70.0/source/boost_1_70_0.tar.bz2
RUN cd /libs; tar -xvf boost_1_70_0.tar.bz2
RUN mkdir /boost && cd /libs/boost_1_70_0 && ./bootstrap.sh --with-libraries=program_options,graph && ./b2 install -j3 -q
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/

LABEL Description="This image contains just my executable so I can limit its memory usage"

# This also creates the dir
WORKDIR /lpa
COPY cmake-build-release/disjoint_CBSH /lpa/cmake-build-release/
COPY cmake-build-debug/disjoint_CBSH /lpa/cmake-build-debug/
