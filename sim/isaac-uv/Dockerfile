FROM nvidia/cuda:12.6.3-base-ubuntu22.04

# --- 1. 用户与权限配置 ---
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -G root -m -s /usr/local/bin/zsh ${USERNAME} \
    && apt-get update && apt-get install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]

# --- 2. 环境变量配置 ---
ENV MAMBA_ROOT_PREFIX=/usr/local \
    UV_SYSTEM_PYTHON=1 \
    ISAACSIM_PATH=/isaac-sim 

# --- 3. 安装包管理工具与 GUI 核心依赖 (APT) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates tar bzip2 unzip \
    libatomic1 libgomp1 libglib2.0-0 libnghttp2-14 \
    libegl1 libgl1 libglu1-mesa libglx0 libsm6 libxi6 libxrandr2 libxt6 \
    # 渲染与窗口支持
    libvulkan1 vulkan-tools libx11-6 libxau6 libxcb1 libxdmcp6 libxext6 \
    && curl -LsSf https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C /usr/bin/ --strip-components=1 bin/micromamba \
    && apt-get -y autoremove \
    #清理中间缓存
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    #给权限!重要
    && chown -R ${USERNAME}:${USER_GID} /usr/local
# --- 合并官方所有驱动配置逻辑 ---
RUN mkdir -p /usr/share/glvnd/egl_vendor.d /etc/vulkan/icd.d /etc/vulkan/implicit_layer.d \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_mesa.so.0"}}\n' > /usr/share/glvnd/egl_vendor.d/50_mesa.json \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.194"}}\n' > /etc/vulkan/icd.d/nvidia_icd.json \
    && printf '{"file_format_version":"1.0.0","layer":{"name":"VK_LAYER_NV_optimus","type":"INSTANCE","library_path":"libGLX_nvidia.so.0","api_version":"1.3.194","implementation_version":"1","description":"NVIDIA Optimus layer","functions":{"vkGetInstanceProcAddr":"vk_optimusGetInstanceProcAddr","vkGetDeviceProcAddr":"vk_optimusGetDeviceProcAddr"},"enable_environment":{"__NV_PRIME_RENDER_OFFLOAD":"1"},"disable_environment":{"DISABLE_LAYER_NV_OPTIMUS_1":""}}}\n' > /etc/vulkan/implicit_layer.d/nvidia_layers.json 
# --- 合并所有相关的环境变量 ---
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
USER ${USERNAME}
# --- 4. Micromamba 强制安装系统级工具 (禁止环境隔离) ---
# 安装 Python 3.11 (Isaac Sim 5.1 必需) 及开发工具到 /usr
RUN micromamba install -y  -c conda-forge \
    python=3.11 uv\ 
    zsh git wget cmake ninja openssh \
    && micromamba clean --all --yes

# --- 5. 使用 UV 安装 Isaac Sim (前置安装) ---
# 接受 EULA 环境变量 (根据 NVIDIA 要求)
ENV ACCEPT_EULA=Y
USER ${USERNAME}
RUN uv pip install --system --no-cache\
    --extra-index-url https://pypi.nvidia.com \
    "isaacsim[all,extscache]==5.1.0"

# --- 6. 开发环境配置 (Oh My Zsh & Starship) ---

WORKDIR /home/${USERNAME}

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && git clone https://github.com/esc/conda-zsh-completion ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/conda-zsh-completion \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting conda-zsh-completion)/' ~/.zshrc \
    && curl -sS https://starship.rs/install.sh | sudo sh -s -- -y \
    && echo 'eval "$(starship init zsh)"' >> ~/.zshrc \
    && mkdir -p ~/.config \
    && printf '[username]\nshow_always = true\nstyle_user = "bold #af87ff"\nformat = "[$user]($style) "' > ~/.config/starship.toml

# --- 7. 延迟加载 micromamba 索引 (仅保留命令补全，不涉及环境激活) --
RUN git clone https://github.com/isaac-sim/IsaacLab.git \
    && cd IsaacLab && ./isaaclab.sh --install sb3 rsl_rl || true

CMD ["/usr/bin/zsh"]