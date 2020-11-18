# ROS2ワークスペースの作成

ROS2のワークスペースを作成する手順の覚書です．

基本的にROS2の[公式チュートリアル](https://index.ros.org/doc/ros2/Tutorials/)に従っています．

## 環境

- Ubuntu 18.04 LTS
- ROS2 Dashing Diademata

## やること

ROS2ではワークスペースの概念として，アンダーレイ(underlay)とオーバーレイ(overlay)が追加されました．
ROS1の頃とは少し異なった概念になっているので，しっかり理解した上でROS2の開発に臨みましょう．

## 事前知識

### ワークスペースとは

ROSにおけるワークスペースとは，「ROSのパッケージが入っているディレクトリ」のことを指します．

ROS1では原則として，1つのワークスペースしか持つことができませんでした．
しかし，ROS2では複数のワークスペースを持つことができるようになり，用途・目的別にワークスペースを分割して開発を進めることが推奨されるようになりました．

複数のワークスペースを作成できるようになったことで，新たに「アンダーレイ(underlay)」と「オーバーレイ(overlay)」という概念が導入されました．

### underlayとoverlay

`underlay`は大雑把に言えばROS2の本体のことです．
ROS2公式ドキュメントの[Using colcon to build packages](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)のページで`underlay`について言及されています．

>It is important that we have sourced the environment for an existing ROS 2 installation that will provide our workspace with the necessary build dependencies for the example packages. This is achieved by sourcing the setup script provided by a binary installation or a source installation, ie. another colcon workspace (see Installation). We call this environment an underlay.

ROS2のコマンドやツール，`apt`コマンドでインストールしたサンプルパッケージ等，ROS2のコアとなるパッケージが存在する基礎的なワークスペースを「`underlay`」と呼びます．

一方で，ユーザーが作成し，ユーザー独自のパッケージを配置するワークスペースを「`overlay`」と呼びます．
`overlay`は`underlay`の上に作成されます．  
ユーザーが作成したパッケージを使用する際は，`underlay`を読み込んだ上で`overlay`を読み込みます．
複数の`overlay`を使用する場合は，`underlay`を読み込んだ上でそれぞれの`overlay`を読み込みます．

## 作業手順

では実際にワークスペースを作成してみましょう．
このワークスペースはユーザーによって作成されるので，`overlay`に当たります．

### underlayを読み込む

ワークスペースの作成には`colcon`コマンドを使う必要があります．
`colcon`コマンドはROS2のコマンドなので，`underlay`を読み込まなければなりません．

次のコマンドを実行して，`underlay`を読み込みます．

```bash
source /opt/ros/dashing/setup.bash
```

これで，`colcon`コマンドや`ros2`コマンドが使用できるようになりました．

ただし，これらのコマンドが使用できるのは，`underlay`を読み込んだターミナル(上の`source`コマンドを実行したターミナル)だけです．
ターミナルを閉じてしまったり，別の新しいターミナルを開いた場合は，もう一度上のコマンドを実行する必要があります．

### ワークスペースにするディレクトリを作成する

ワークスペースにするディレクトリを作成します．
ワークスペースはホームディレクトリ直下に作るのが一般的です．

ROS1ではワークスペース名を`catkin_ws`にするのが慣例的でしたが，ROS2では名前に縛りはありません．
ただし，ワークスペースの目的を端的に表すような名前を付けると便利です．

ここでは`dev_ws`という名前で作ることにしましょう．

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

`dev_ws`という名前のディレクトリを作り，その中に`src`という名前のディレクトリを作っています．
ワークスペースは`src`という名前のディレクトリを持つことが必須の条件となっているため，このようにしています．

### サンプルパッケージをダウンロードする

作成したワークスペースの`src`ディレクトリに，サンプルパッケージをダウンロードしてビルドしてみます．

ROS公式のサンプルパッケージが[GitHub](https://github.com/ros/ros_tutorials/tree/dashing-devel)にあるので，これをダウンロードします．

ダウンロードには`git`コマンドを使用します．次のコマンドを実行してください．

```bash
cd ~/dev_ws/src
git clone https://github.com/ros/ros_tutorials.git -b dashing-devel
```

これで，`src`ディレクトリの中に`ros_tutorials`ディレクトリがダウンロードされました．  
`ros_tutorials`ディレクトリの中身を確認してみましょう．次のコマンドを実行します．

```bash
ls ros_tutorials
```

次のように表示されるはずです．

```console
roscpp_tutorials  rospy_tutorials  ros_tutorials  turtlesim
```

このうち，`turtlesim`以外のパッケージはビルド対象から外れています．
各パッケージのディレクトリに`COLCON_IGNORE`という名前のファイルが置かれているため，ビルドを実行しても無視されます．

### 依存関係の解決

ROSパッケージをビルドする際は，ビルド対象となる全てのパッケージの依存関係を解決しておく必要があります．
依存関係が解決していないとビルドが失敗してしまうため，ビルド前に必ずチェックしておくことが推奨されます．

`rosdep`コマンドを使うことで，パッケージの依存関係の解決を簡単に行うことができます．
次のコマンドは，ワークスペースの`src`ディレクトリ内にある全てのパッケージに対して依存関係の解決を試みるコマンドです．

```bash
cd ~/dev_ws
rosdep install -i --from-path src --rosdistro foxy
```

もしパッケージが不足している場合は，Debianパッケージをインストールしようとします．  
パッケージが充足している場合は，次のような文が表示されます．

```console
#All required rosdeps installed successfully
```

### ワークスペースをビルドする

いよいよワークスペースをビルドします．
ビルドには`colcon`コマンドを使用します．
ワークスペースディレクトリに移動し，ビルドを実行します．

```bash
cd ~/dev_ws
colcon build
```

ビルドが実行されると，次のようなメッセージが表示されます．

```console
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```

ビルドにかかる時間はマシンスペックに影響されます．
Raspberry Piのような性能が限られたマシンの場合，数分以上かかってしまう場合もあります．

ビルドが終了すると，ワークスペースには`build/`，`install/`，`log/`，ディレクトリが生成されます．  
`build/`ディレクトリにはビルドによって生成された実行可能ファイルが置かれます．
`install/`ディレクトリには，`overlay`を読み込むためのセットアップスクリプトが置かれます．

### overlayを読み込む

ワークスペースをビルドしただけでは，その中のROSパッケージが使えるようにはなりません．
ワークスペース内のROSパッケージを使用可能にするには，`overlay`を読み込む必要があります．

ここで注意すべきなのは，`overlay`を読み込む際は，`underlay`を**読み込んでいないターミナル**で行う必要があるという事です．
`underlay`を読み込んだターミナルで`overlay`を読み込んだり，`overlay`を読み込んだターミナルでビルドを実行すると，複雑な問題が発生する可能性があります．

言い換えれば，「ビルドを実行するターミナルと，ワークスペースにあるROSパッケージを使用するターミナルは完全に分離しておくべき」という事になります．
ビルドを実行するターミナルでは`underlay`だけを読み込んでおき，逆にROSパッケージを使用するターミナルでは`overlay`だけを読み込むようにしておくと良いでしょう．

では`overlay`を読み込んでみます．ビルドを実行した(`underlay`を読み込んでいた)ターミナルとは**別の**ターミナルを新たに開き，まず`underlay`を読み込みます．

```bash
source /opt/ros/dashing/setup.bash
```

そして，ワークスペースに移動し，`local_setup.bash`スクリプトを読み込むことで`overlay`をセットします．

```bash
cd ~/dev_ws
source install/local_setup.bash
```

これで`dev_ws`内にあるパッケージを環境に読み込むことができました．
この状態では，`turtlesim`パッケージを実行することができるようになります．

試しに実行してみましょう．次のコマンドを実行することで，`turtlesim`パッケージの`turtlesim_node`を起動することができます．

```bash
ros2 run turtleseim turtlesim_node
```

### Tips: 2つのoverlayセットアップスクリプト

作成したワークスペースの`install/`ディレクトリを見ると，名前の異なる2つのセットアップスクリプトが存在することがわかります．

- `setup.bash`
- `local_setup.bash`

この2つは何が違うのでしょうか？  
`local_setup.bash`は，そのワークスペースの中にあるパッケージ**のみ**を環境へ追加します．
一方で，`setup.bash`はそのワークスペースの中にあるパッケージに加えて，このセットアップスクリプトが生成された時点(つまりワークスペースをビルドした時点)での`underlay`を読み込み，両方を使えるようにします．

すなわち，`overlay`の`setup.bash`を読み込むことは，「`underlay`を読み込んだ後に`overlay`の`local_setup.bash`を読み込むこと」と同じであるという事になります．

ただ1つ異なるのは，`overlay`の`setup.bash`が読み込む`underlay`は，そのワークスペースがビルドされた時点での`underlay`であるという事です．
このことを考えると，複数のワークスペースでパッケージを並列して開発する際は，`overlay`の読み込みには`local_setup.bash`を読み込むようにした方が良いかもしれません．

ROS2のセットアップのベストプラクティスはまだ分かりませんが，少なくとも`install/setup.bash`を読み込むのは控えたほうが良さそうです．

ちなみに筆者は`~/.bashrc`に以下のような設定を記述しています．

```bash
source /opt/ros/dasing/setup.bash
alias ulay="source /opt/ros/dashing/setup.bash"
alias olay="source ./install/local_setup.bash"
```

シェルを立ち上げたときに`underlay`を自動で読み込む設定の他，`underlay`を読み込むコマンドと`overlay`の`local_setup.bash`を読み込むコマンドのエイリアスを設定しています．  
エイリアスを設定しておくことで，`ulay`と実行するだけで`underlay`の読み込みが行えるようになり，またワークスペースのルートディレクトリにいる状態で`olay`と実行すると，そのワークスペースの`local_setup.bash`を読み込めるようになります．

こうすることで，ワークスペースをビルドした後の流れがスムーズになります．
例えば以下のような例です．

```bash
ulay          # underlayはシェル立ち上げ時に読み込まれているので必要ないけど一応
cd ~/dev_ws   # ワークスペースのルートに移動
colcon build  # パッケージをビルド
olay          # ビルドしたパッケージを環境に追加する
```

今後，`underlay`と`overlay`の管理方法についてのベストプラクティスが提案されるかもしれません．
ROS2は未だ開発途上のソフトウェアなので，常に最新の情報をチェックするよう心がけましょう．
