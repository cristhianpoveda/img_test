# parameters
ARG REPO_NAME="img_test"
ARG DESCRIPTION="c++ test"
ARG MAINTAINER="Cristhian Daniel Poveda Bravo (cristhianpoveda12@gmail.com)"
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG ICON="cube"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH=arm32v7
ARG DISTRO=daffy
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG BASE_IMAGE=dt-ros-commons
ARG LAUNCHER=default

# define base image
ARG DOCKER_REGISTRY=docker.io
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as BASE

# recall all arguments
ARG ARCH
ARG DISTRO
ARG REPO_NAME
ARG DESCRIPTION
ARG MAINTAINER
ARG ICON
ARG BASE_TAG
ARG BASE_IMAGE
ARG LAUNCHER

# check build arguments
RUN dt-build-env-check "${REPO_NAME}" "${MAINTAINER}" "${DESCRIPTION}"

# define/create repository path
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
RUN mkdir -p "${REPO_PATH}"
RUN mkdir -p "${LAUNCH_PATH}"
WORKDIR "${REPO_PATH}"

# keep some arguments as environment variables
ENV DT_MODULE_TYPE "${REPO_NAME}"
ENV DT_MODULE_DESCRIPTION "${DESCRIPTION}"
ENV DT_MODULE_ICON "${ICON}"
ENV DT_MAINTAINER "${MAINTAINER}"
ENV DT_REPO_PATH "${REPO_PATH}"
ENV DT_LAUNCH_PATH "${LAUNCH_PATH}"
ENV DT_LAUNCHER "${LAUNCHER}"

# ROS key
RUN sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
##
RUN apt-get update

# # install apt dependencies
COPY ./dependencies-apt.txt "${REPO_PATH}/"
RUN dt-apt-install ${REPO_PATH}/dependencies-apt.txt

RUN git clone --depth=1 https://github.com/opencv/opencv.git && \
    cd ./opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D BUILD_ZLIB=ON \
          -D BUILD_OPENMP=ON \
          -D BUILD_TIFF=OFF \
          -D BUILD_OPENJPEG=OFF \
          -D BUILD_JASPER=OFF \
          -D BUILD_OPENEXR=OFF \
          -D BUILD_WEBP=OFF \
          -D BUILD_TBB=ON \
          -D BUILD_IPP_IW=OFF \
          -D BUILD_ITT=OFF \
          -D WITH_OPENMP=ON \
          -D WITH_OPENCL=OFF \
          -D WITH_AVFOUNDATION=OFF \
          -D WITH_CAP_IOS=OFF \
          -D WITH_CAROTENE=OFF \
          -D WITH_CPUFEATURES=OFF \
          -D WITH_EIGEN=OFF \
          -D WITH_GSTREAMER=ON \
          -D WITH_GTK=OFF \
          -D WITH_IPP=OFF \
          -D WITH_HALIDE=OFF \
          -D WITH_VULKAN=OFF \
          -D WITH_INF_ENGINE=OFF \
          -D WITH_NGRAPH=OFF \
          -D WITH_JASPER=OFF \
          -D WITH_OPENJPEG=OFF \
          -D WITH_WEBP=OFF \
          -D WITH_OPENEXR=OFF \
          -D WITH_TIFF=OFF \
          -D WITH_OPENVX=OFF \
          -D WITH_GDCM=OFF \
          -D WITH_TBB=ON \
          -D WITH_HPX=OFF \
          -D WITH_EIGEN=OFF \
          -D WITH_V4L=ON \
          -D WITH_LIBV4L=ON \
          -D WITH_VTK=OFF \
          -D WITH_QT=OFF \
          -D BUILD_opencv_python3=OFF \
          -D BUILD_opencv_java=OFF \
          -D BUILD_opencv_gapi=OFF \
          -D BUILD_opencv_objc=OFF \
          -D BUILD_opencv_js=OFF \
          -D BUILD_opencv_ts=OFF \
          -D BUILD_opencv_dnn=OFF \
          -D BUILD_opencv_calib3d=OFF \
          -D BUILD_opencv_objdetect=OFF \
          -D BUILD_opencv_stitching=OFF \
          -D BUILD_opencv_ml=OFF \
          -D BUILD_opencv_world=OFF \
          -D BUILD_EXAMPLES=OFF \
          -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
          -D OPENCV_ENABLE_NONFREE=OFF \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D INSTALL_C_EXAMPLES=OFF \
          -D INSTALL_PYTHON_EXAMPLES=OFF .. &&\
    make -j4 && \
    make install && \
    apt-get update && \
    rm -rf ./opencv

# # install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${REPO_PATH}/"
RUN python3 -m pip install  -r ${REPO_PATH}/dependencies-py3.txt

# install launcher scripts
COPY ./launchers/. "${LAUNCH_PATH}/"
COPY ./launchers/default.sh "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.module.type="${REPO_NAME}" \
    org.duckietown.label.module.description="${DESCRIPTION}" \
    org.duckietown.label.module.icon="${ICON}" \
    org.duckietown.label.architecture="${ARCH}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"
# <== Do not change the code above this line
# <==================================================
# copy the source code
COPY ./packages "${REPO_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/