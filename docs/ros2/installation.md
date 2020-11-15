# ROS2の環境構築の覚書

LinuxマシンにROS2をインストールして開発環境を整える手順の覚書です．

基本的にROS2の[公式チュートリアル](https://index.ros.org/doc/ros2/Tutorials/)に従っています．

## 環境

- Ubuntu 18.04 LTS
- ROS2 Dashing Diademata

## やること

ROS2 Dashingのインストールと環境構築を行います．  
ROS2のビルド済みバイナリを`apt`コマンドを使ってDebianパッケージとしてインストールします．

## 作業手順

[公式ドキュメント](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)に従ってインストールを行います．

### localeの設定

ROS2のアプリケーションは`UTF-8`のみサポートしているため，Ubuntuの言語設定を`UTF-8`にしておく必要があります．
次のコマンドは，言語設定を英語の`UTF-8`にする設定です．

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

日本語の言語設定であっても，`UTF-8`をサポートしているならばROS2は動作します．
Ubuntuにおいて日本語ロケールを設定するには，次のコマンドを実行します．

```bash
sudo apt update
sudo apt install language-pack-ja
sudo update-locale LANG=ja_JP.UTF-8
```

### ROS2のリポジトリを登録する

ROS2のリポジトリサーバを登録します．
まずはこの作業に必要となるソフトウェアをインストールします．

```bash
sudo apt update
sudo apt install curl gnupg2 lsb-release
```

次に，ROS2のリポジトリのGPGキーを`apt`(Ubuntuにおけるパッケージ管理アプリケーション)に登録します．

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

そして，ROS2のリポジトリを`apt`のリストに追加します．

```bash
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### ROS2のインストール

ROS2のリポジトリの登録が終わったら，パッケージリストを更新しておきます．

```bash
sudo apt update
```

いよいよROS2をインストールします．
ROS2をインストールする場合は，フルインストール(ROS2本体の他に，GUIアプリケーションやデモンストレーションパッケージが入っている)が推奨されているようです．
デスクトップ環境が構築されているマシンでのみ，フルインストールを行ってください．

```bash
sudo apt install ros-dashing-desktop
```

Dockerコンテナで環境構築を行う場合や，Raspberry Pi等デスクトップ環境が無いマシンでは，ROS2の本体のみをインストールします．

```bash
sudo apt install ros-dashing-base
```

インストールには時間がかかります．マシンがスリープしないように気を付けて，気長に待ちましょう．

### 各種ツールのインストール

ROS2で開発を行う際に必要となるツール群のインストールを行います．

```bash
sudo apt install python3 python3-pip python3-colcon-common-extensions python3-rosdep python3-argcomplete
```

`rosdep`のセットアップをしておきます．以下のコマンドを実行してください．

```bash
sudo rosdep init
rosdep update
```

ここまでの作業で，ROS2のインストール作業は完了です．
次は環境設定を行います．

## 環境設定

公式ドキュメントの[Configuring your ROS2 environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/)のページを参考に作業を行います．

### ROS2の環境構築スクリプトの読み込み

ROS2のコマンドをターミナル上で使用するには，環境構築スクリプトを読み込む必要があります．

```bash
source /opt/ros/dashing/setup.bash
```

セットアップスクリプトは，ROS2が動作するための各種環境変数のセッティングを行ってくれます．
このセットアップスクリプトを読み込まなければ，`ros2`コマンドを使用することはできません．

ここで，公式ドキュメントでは，セットアップスクリプトの読み込みを自動化する作業を行っていますが，これは**推奨しません**．

詳しい話はワークスペース作成のページで説明しますが，ROS2には`underlay`と`overlay`の2つの環境レイヤーが存在します．
`underlay`はROS2のコアとなる環境，`overlay`は個別のワークスペースの環境です．
`/opt/ros/dashing/setup.bash`は`underlay`のセットアップスクリプトであり，これを読み込んだ状態で各ワークスペースのセットアップスクリプトを読み込むと，複雑な問題が発生する可能性があると言及されています([Creating a workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)のページ)．

> Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.
>
> In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:

`underlay`のセットアップスクリプトを自動読み込みすると，意図しないスクリプト読み込みが発生してトラブルが起こる恐れがあるため，少々面倒ですがセットアップスクリプトの読み込みは手動で行うようにしましょう．

### 環境変数のチェック

セットアップスクリプトを読み込んだら，ROS2の環境変数のチェックをしてみましょう．
この作業は必ずしもやる必要はありません．

環境変数をチェックするには，次のコマンドを実行します．

```bash
printenv | grep -i ROS
```

このコマンドを実行すると，以下のように表示されます．

```console
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=dashing
```

### ROS_DOMAIN_IDの設定

研究室やオフィス等，1つのネットワーク上でROS2を実行する複数のマシンが存在する環境では，それぞれのマシンが干渉することを防ぐため，`ROS_DOMAIN_ID`という環境変数を設定する必要があります．

`ROS_DOMAIN_ID`は0から232の整数を持つ環境変数で，この変数の値が同じであるマシン同士は，ROS2のネットワークを共有することができ，通信を行えるようになります．
逆に，異なる値を設定したマシン同士では通信を行えなくなります．

同じネットワークに属するマシン同士の意図しない通信を防ぐためにも，一意な値を`ROS_DOMAIN_ID`にセットしておくことをお勧めします．

環境変数の設定は`export`コマンドで行います．次のコマンドは，IDを121に設定する例です．

```bash
export ROS_DOMAIN_ID=121
```

イコールの間にスペースを挟んではいけません．

このコマンドを実行することでIDが設定されますが，マシンを再起動すると設定は失われてしまいます．  
設定を永続化したい場合は，次のコマンドを実行します．

```bash
echo "export ROS_DOMAIN_ID=121" >> ~/.bashrc
```

これでROS2の環境設定は完了です．
