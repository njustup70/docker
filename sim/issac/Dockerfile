FROM nvcr.io/nvidia/isaac-sim:5.1.0
USER root
# --- 参数与环境变量配置 ---
# 1. 合并所有构建参数 (ARG)
ARG USERNAME=Elaina \
    USER_UID=1000 \
    USER_GID=1000
# 2. 合并所有环境变量 (ENV)
ENV MAMBA_ROOT_PREFIX=/opt/conda \
    PATH=/opt/conda/bin:$PATH \
    UV_SYSTEM_PYTHON=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    zsh sudo ca-certificates bzip2 \
    && rm -rf /var/lib/apt/lists/*
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C /usr/bin/ --strip-components=1 micromamba

#增加用户
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /usr/bin/zsh \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && mkdir -p $MAMBA_ROOT_PREFIX \
    && chown -R $USERNAME:$USER_GID $MAMBA_ROOT_PREFIX $ISAAC_SIM_PATH

# 6. 安装 Oh My Zsh & Starship (提升开发体验)
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && git clone https://github.com/esc/conda-zsh-completion ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/conda-zsh-completion \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting conda-zsh-completion)/' ~/.zshrc \
    && curl -sS https://starship.rs/install.sh | sh -s -- -y \
    && echo 'eval "$(starship init zsh)"' >> ~/.zshrc \
    && mkdir -p ~/.config \
    && printf '[username]\nshow_always = true\nstyle_user = "bold #af87ff"\nformat = "[$user]($style) "' > ~/.config/starship.toml
# 安装issac lab
CMD ["/usr/bin/zsh"]
