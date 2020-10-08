# Dockerのインストール手順

## 環境

- Ubuntu 18.04 LTS Desktop

## Docker本体のインストール手順

Docker Engine - Community for UbuntuをUbuntu 18.04 LTSにインストールします．

インストール手順は[Docker公式ドキュメント](https://docs.docker.com/engine/install/ubuntu/)に書かれている通りにすればOKです．

### 古いバージョンのDockerをアンインストールする

古いバージョンのDockerがマシンにインストールされている場合は，アンインストールしてから作業します．

```bash
sudo apt remove docker docker-engine docker.io containerd runc
```

### 依存パッケージのインストール

Dockerに必要な依存パッケージをインストールします．

```bash
sudo apt update
sudo apt install apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
```

### リポジトリの鍵の設定

Dockerのリポジトリの鍵を入手します．

```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

入手した鍵のフィンガープリントを確認しておきます(必ずしもやる必要はありません)．

```console
$ sudo apt-key fingerprint 0EBFCD88

pub   rsa4096 2017-02-22 [SCEA]
      9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88
uid           [ unknown] Docker Release (CE deb) <docker@docker.com>
sub   rsa4096 2017-02-22 [S]
```

安定版のDockerリポジトリを登録します．

```bash
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
```

### インストール

Dockerをインストールします．

```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
```

インストールが完了したら，サンプルイメージを起動してみましょう．

```bash
sudo docker run hello-world
```

サンプルイメージの起動に成功したら，インストール作業は完了です．

### 一般ユーザにDocker起動権限を渡す

Dockerコンテナは本来，rootユーザーでなければ起動することができません．
そのため，一般ユーザーがDockerを利用するためには`sudo`を付けてコマンドを実行する必要があります．

一般ユーザーを`docker`グループに追加することで，一般ユーザーであっても`sudo`無しでDockerを起動することができます．

```bash
sudo usermod -aG docker ${USER}
```

ただし，**一般ユーザーをdockerグループに追加することで，コンテナがホストマシン上のroot権限を取得できてしまうというセキュリティ上のリスクが存在します．**
実験・開発用なら問題無いかもしれませんが，セキュリティリスクが存在するという事を念頭に置いておきましょう．

## docker-composeのインストール手順

複数のコンテナを起動する際や，沢山のオプションを与えてコンテナを起動する際に便利なツールとして，[docker-compose](https://docs.docker.com/compose/)があります．

インストール手順は[公式サイト](https://docs.docker.com/compose/install/)に書かれています．

### docker-composeのダウンロード

Linuxの場合，以下のコマンドを実行してdocker-compose最新版をダウンロードします．

```bash
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
```

このコマンドによってバージョン1.27.4のdocker-composeがダウンロードされます．

### 実行バイナリに権限を付与する

ダウンロードしたバイナリに実行権限を付与します．

```bash
sudo chmod +x /usr/local/bin/docker-compose
```

これでインストールは完了です．インストールできたことを確認しましょう．

```bash
docker-compose --version
```
