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
    sudo build-essential cmake git clangd openssh-client pkg-config curl\
    && rm -rf /var/lib/apt/lists/* && apt-get clean
# 2. 创建用户并配置权限
RUN groupadd --gid $USER_GID $USERNAME  \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /usr/bin/bash\
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && usermod -aG root $USERNAME \
    && echo 'Defaults !env_reset' >> /etc/sudoers \
    && echo 'Defaults env_keep += "PATH"' >> /etc/sudoers \
    && echo 'Defaults secure_path="/opt/conda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"' >> /etc/sudoers \
    && echo "umask 002" | tee -a /etc/profile /etc/bash.bashrc \
    && chown -R $USERNAME:$USER_GID $MAMBA_ROOT_PREFIX\
    && chmod -R 775 $MAMBA_ROOT_PREFIX 
# 3. 配置并安装 Micromamba 环境 (仅保留 ROS 相关和辅助工具)
USER $USERNAME
# 删除了 c-compiler, cxx-compiler, cmake, git，改用系统级的
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /opt/conda/.mambarc && \
    micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs\
    colcon-common-extensions \
    colcon-mixin \
    && ln -s /usr/bin/python /opt/conda/bin/python \
    && micromamba clean --all --yes
#安装其他开发工具
RUN micromamba install -y \
    ninja ccache mold zsh uv gdb  vim \
    && micromamba clean --all --yes 
#指定使用 micromamba 运行后续命令
SHELL ["micromamba", "run", "-n", "base", "/bin/bash", "-c"]
# # 6. 使用 UV 安装 Python 分析工具
RUN uv pip  install  viztracer py-spy
# 安装可视化工具所需的 OpenGL 库
RUN micromamba install -y -n base \
    libgl-devel \
    mesalib \
    libglvnd \
    && micromamba clean -afy
#增加其他ROS包 与python补全
RUN micromamba install -y -n base \
    argcomplete \   
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    && micromamba clean --all --yes
# --- 运行时配置 ---

# 安装 Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && git clone https://github.com/esc/conda-zsh-completion ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/conda-zsh-completion \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting conda-zsh-completion)/' ~/.zshrc \
    && curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init zsh)"' >> ~/.zshrc \
    && mkdir -p ~/.config \
    && printf '[username]\nshow_always = true\nstyle_user = "bold #af87ff"\nformat = "[$user]($style) "' > ~/.config/starship.toml
WORKDIR /home/$USERNAME
#直接将micromamba的环境变量添加到系统环境变量中，确保在任何 shell 中都能正确使用 micromamba 环境
ENV LD_LIBRARY_PATH=$MAMBA_ROOT_PREFIX/lib:$LD_LIBRARY_PATH
ENV CONDA_PREFIX=$MAMBA_ROOT_PREFIX
ENV CONDA_DEFAULT_ENV=base
RUN echo "source ${MAMBA_ROOT_PREFIX}/setup.bash" >> ~/.bashrc \
    && cat <<-'EOF' >> ~/.zshrc
# 劫持 source 函数以修复 ROS2 补全覆盖问题
source() {
    builtin source "$@"
    if [[ "$*" == *"setup.zsh"* ]]; then
        if command -v register-python-argcomplete > /dev/null; then
            eval "$(register-python-argcomplete ros2)"
            eval "$(register-python-argcomplete colcon)"
        fi
    fi
}
# 加载 Mamba 环境
source "${MAMBA_ROOT_PREFIX}/setup.zsh"
EOF
#解决zsh中补全问题
CMD ["/bin/bash"]
