FROM gcc

RUN apt update && apt install -y gdb gdbserver

# This also creates the dir
WORKDIR /libs
RUN cd /libs && git clone https://github.com/google/glog.git && cd glog && ./autogen.sh && ./configure && make && \
 make install && cd /libs && rm glog -rf
RUN cd /libs && wget https://dl.bintray.com/boostorg/release/1.72.0/source/boost_1_72_0.tar.bz2 && \
 tar -xvf boost_1_72_0.tar.bz2 && cd /libs/boost_1_72_0 && ./bootstrap.sh --with-libraries=program_options,graph && \
 ./b2 install -j3 -q && cd /libs && rm boost_1_72_0.tar.bz2 && rm boost_1_72_0 -rf
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
RUN cd /libs; wget https://packages.gurobi.com/8.1/gurobi8.1.1_linux64.tar.gz
RUN cd /libs; tar xvfz gurobi8.1.1_linux64.tar.gz
ENV GUROBI_HOME=/libs/gurobi811/linux64
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib/
ENV PATH=$PATH:$GUROBI_HOME/bin

# Add a Gurobi license file that points to the BGU token server
RUN echo "TOKENSERVER=132.72.65.143" > /root/gurobi.lic
# (Uncomment the following line if the token server is running locally)
#RUN echo "TOKENSERVER=localhost" > /root/gurobi.lic

LABEL Description="This image contains just my executable and its dependencies so I can limit its memory usage"

# This also creates the dir
WORKDIR /lpa
COPY ECBSH_no_lpa /lpa/
COPY ECBSH_no_lpa_latest_conflict /lpa/
COPY ECBSH_no_lpa_up_and_down /lpa/
COPY ECBSH_no_lpa_up_and_down_latest_conflict /lpa/
COPY ECBSH_lpa /lpa/
COPY ECBSH_lpa_not_latest_conflict /lpa/
COPY ECBSH_lpa_up_and_down /lpa/
COPY ECBSH_lpa_up_and_down_not_latest_conflict /lpa/
COPY IDCBSH_no_lpa /lpa/
COPY IDCBSH_no_lpa_latest_conflict /lpa/
COPY IDECBSH_lpa_with_lpmdd_and_path_repair /lpa/
#COPY IDECBSH_lpa_with_lpmdd_and_path_repair_not_latest_conflict /lpa/

#COPY cmake-build-release/disjoint_CBSH /lpa/cmake-build-release/
#COPY cmake-build-debug/disjoint_CBSH /lpa/cmake-build-debug/
