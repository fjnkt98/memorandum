# catkin_tools 入門

## Catkin Command Line Toolsとは

Catkin Command Line Toolsとは，ROS1における新しいコマンドラインビルドツールのことです．catkin_makeに代わる新しいツールとして提供されています．

こちらのコマンドツールの方がモダンなので，今からROS1を始める際にはcatkin_toolsを使用することをお勧めします．

この記事では，頻繁に使用するコマンドについての簡単な説明をします．
コマンドについての詳細な解説は，[catkin_toolsの公式サイト](https://catkin-tools.readthedocs.io/en/latest/index.html)に記載されています．より詳細な情報が欲しい際はそちらを参考にしてください．

## catkin_toolsのインストール

catkin_toolsはデフォルトでROSに含まれていません．そのため，ROSをインストールした後に追加でインストールする必要があります．

```bash
sudo apt install python-catkin-tools
```

## 使い方

### ワークスペースの初期化

ROSワークスペースの初期化は`catkin init`コマンドで行うことができます．
ワークスペース名は自由ですが，"catkin_ws"という名前にするのが慣例となっています．

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

### パッケージの作成

パッケージの作成は`catkin create pkg`コマンドで行えます．

パッケージに付けたい名前をコマンド引数として与えることで，パッケージ名を指定することができます．

ただし，ROSパッケージの名前は全てlower_case(小文字の単語をアンダースコアで繋ぐ)で命名する必要があります．

```bash
catkin create pkg PACKAGE_NAME --catkin-deps roscpp std_msgs
```

依存パッケージを追加するには，`--catkin-deps`オプションの後に続けて依存パッケージを並べて記述します．

また，オプションによってライセンスやパッケージ管理者の名前，ライセンスを指定することもできます．これらの情報は作成されたパッケージ内にある`package.xml`に書かれるものなので，コマンドで指定しなくても後から編集できます．

### パッケージのビルド

#### 全てのパッケージをビルド

`catkin build`コマンドによって，ワークスペース内のパッケージをビルドすることができます．
catkin_makeと違い，ワークスペース内のどのディレクトリにいてもパッケージをビルドすることができます．

ワークスペース内のパッケージを全てビルドするには，単に`catkin build`コマンドを実行します．

```bash
cd ~/catkin_ws
catkin build
```

#### 単体パッケージのビルド

catkin_toolsでは，特定のパッケージを個別にビルドすることもできます．

パッケージの名前をコマンド引数として指定することで，個別にビルドするパッケージを選べます．

```bash
catkin build PACKAGE_NAME
```

もしくは，個別にビルドしたいパッケージのディレクトリの中にいる場合は，`--this`オプションを付けるだけで単体ビルドを行うことができます．

```bash
cd /path/to/package
catkin build --this
```

---

この他にも様々なコマンドやオプションが用意されています．もっと知りたい場合は[catkin_toolsの公式サイト](https://catkin-tools.readthedocs.io/en/latest/index.html)を参考にしてください．
