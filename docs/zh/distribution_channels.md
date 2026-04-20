# 分发渠道（CI 自动化）

本文说明当前启用的 GitHub Releases / PyPI 之外分发渠道。

## 1) APT 仓库（GitHub Pages）

对应 workflow：`.github/workflows/apt-repo-publish.yml`

- 触发方式：
  - Release `published`
  - 或手动触发（`workflow_dispatch`）并传入 `tag`
- 输入资产：
  - 对应 Release 里的 `*.deb`
- 输出：
  - 在 `gh-pages` 分支发布 `/apt` 仓库目录
  - URL 形如：
    - `https://<owner>.github.io/<repo>/apt`

可选签名（生产建议开启）：

- `APT_GPG_PRIVATE_KEY`（ASCII armored 私钥）
- `APT_GPG_PASSPHRASE`（若私钥有口令）

如果未配置签名密钥，workflow 会走 `-skip-signing`（无签名元数据）。

## 包类型与开发人群对应

| 包 / 资产 | 面向开发者 | 典型用途 |
|---|---|---|
| `motorbridge-abi-<tag>-linux-x86_64.deb` | Ubuntu/Debian 上的 C/C++ 机器人开发者 | 安装 ABI（`libmotor_abi`）+ 头文件 + CMake 配置做原生集成 |
| `motorbridge-abi-<tag>-linux-*.tar.gz` / `windows-*.zip` | 非 deb 环境或自定义工具链的 C/C++ 开发者 | 手动解包并链接，适合嵌入式/交叉编译场景 |
| `motorbridge-*.whl` / `motorbridge-*.tar.gz` | Python 应用与工具开发者 | 用 Python SDK 开发上位机、标定脚本、工厂工具 |
| `motor-cli-<tag>-<platform>.tar.gz/.zip` | 测试/运维/工厂工程师 | 不写 SDK 代码，直接做电机扫描和控制 |

## 2) Arch Linux 仓库

通过 [AUR 仓库](https://aur.archlinux.org/pkgbase/motorbridge) 或 [自托管仓库](https://github.com/taotieren/aur-repo) 安装。

- motorbridge/lilac.yaml 自动获取当前仓库 `tag` 并触发版本更新后并推送到 AUR 仓库

```bash
yay -Syu libmotorbridge
yay -Syu python-motorbridge
yay -Syu motorbridge
```

| 包 / 资产 | 面向开发者 | 典型用途 |
|---|---|---|
| `libmotorbridge-<tag>-*.tar.pkg.zst` | Arch Linux 上的 C/C++ 机器人开发者 | 安装 ABI（`libmotor_abi`）+ 头文件 + CMake 配置做原生集成 |
| `python-motorbridge-<tag>-*.tar.pkg.zst` | Python 应用与工具开发者 | 用 Python SDK 开发上位机、标定脚本、工厂工具 |
| `motorbridge-<tag>-*.tar.pkg.zst` | 测试/运维/工厂工程师 | 不写 SDK 代码，直接做电机扫描和控制 |

当前阶段，为保证发布链路“可完全自动化且可预测”，Homebrew 与 Windows 包管理元数据自动化已暂时移除。