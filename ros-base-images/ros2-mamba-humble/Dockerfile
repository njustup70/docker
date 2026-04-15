FROM mambaorg/micromamba:latest
USER root

# --- 参数与环境变量配置 ---
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

# 基础定义
ENV MAMBA_ROOT_PREFIX=/opt/conda
ENV PATH=$MAMBA_ROOT_PREFIX/bin:$PATH

# 针对 ROS 2 的必须变量
ENV ROS_DISTRO=humble

ENV UV_SYSTEM_PYTHON=1
# 1. 安装系统工具与编译依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo build-essential cmake git \ 
    pkg-config \
    && rm -rf /var/lib/apt/lists/*
# 2. 创建用户并配置权限
RUN groupadd --gid $USER_GID $USERNAME  \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME\
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && usermod -aG root $USERNAME \
    && echo 'Defaults !env_reset' >> /etc/sudoers \
    && echo "umask 002" | tee -a /etc/profile /etc/bash.bashrc \
    && chown -R $USERNAME:$USER_GID $MAMBA_ROOT_PREFIX
# 3. 配置并安装 Micromamba 环境 (仅保留 ROS 相关和辅助工具)
# 删除了 c-compiler, cxx-compiler, cmake, git，改用系统级的
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /root/.mambarc && \
    micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs\
    colcon-common-extensions \
    colcon-mixin \
    && micromamba clean --all --yes
#安装其他开发工具
RUN micromamba install -y \
    ninja ccache mold zsh uv gdb curl\
    vim \
    && micromamba clean --all --yes 
#指定使用 micromamba 运行后续命令
SHELL ["micromamba", "run", "-n", "base", "/bin/bash", "-c"]
# # 6. 使用 UV 安装 Python 分析工具
RUN uv pip  install  viztracer py-spy
# 安装可视化工具所需的 OpenGL 库
RUN micromamba install -y -n base \
    libgl-devel \
    mesalib \
    libglvnd
#增加其他ROS包 
RUN micromamba install -y -n base \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp
# --- 运行时配置 ---
USER $USERNAME
# 安装 Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' ~/.zshrc \
    && curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init zsh)"' >> ~/.zshrc \
    && mkdir -p ~/.config \
    && echo '[username]\nshow_always = true\nstyle_user = "bold #af87ff"\nformat = "[$user]($style) "' > ~/.config/starship.toml
WORKDIR /home/$USERNAME
RUN micromamba shell hook --shell bash >> ~/.bashrc \
    && echo "micromamba activate base" >> ~/.bashrc \
    && micromamba shell hook --shell zsh >> ~/.zshrc \
    && echo "micromamba activate base" >> ~/.zshrc \
    && echo "source ${MAMBA_ROOT_PREFIX}/setup.bash" >> ~/.bashrc \
    && echo "source ${MAMBA_ROOT_PREFIX}/setup.zsh" >> ~/.zshrc
CMD ["/bin/bash"]