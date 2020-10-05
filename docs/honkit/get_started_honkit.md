# HonKitの始め方 with GitHub Pages

## 概要

このページでは，[HonKit](https://github.com/honkit/honkit)を使ったWebサイトの作り方を説明します．

また，作成したWebサイトをGitHub Pagesで公開する方法についても解説します．

## 環境

- Windows10 Home version 1909
- Node.js version 12.18.4
- npm version 6.14.6
- HonKit version 3.6.6
- Git version 2.20.1.windows.1

Node.js，npm，Gitはインストール済みであるとします．

また，環境構築やWebページのビルドのためのコマンド実行はWindowsコマンドプロンプト上で(GitコマンドだけはGit Bash上で)行うものとします．

## 作業手順

### GitHubリポジトリの作成

何はともあれGitHubリポジトリを作ります．[GitHub](https://github.com/)にアクセスし，リポジトリ作成画面に移動します．

![Create Repository](images/create_repository.png)

リポジトリ名やDescriptionの欄は適当なものを入れておきます．

Node.jsのプロジェクトなので，「Add .gitignore」のチェックをオンにしておき，テンプレートは「Node」を選択しておきます(Noneと間違わないように注意)．

`README.md`も後々使うので，「Add README.md」のチェックもオンにしておきましょう．

リポジトリが作成できたら，自分のPCにリポジトリをクローンします．

### Node.jsプロジェクトのイニシャライズ

Node.jsプロジェクトを作成します．

先程ローカルにクローンしたリポジトリのディレクトリに移動し，以下のコマンドを実行します．これはコマンドプロンプトで実行します．

```bat
npm init --yes
```

このコマンドによって新しいNode.jsプロジェクトが作られ，ディレクトリには`packages.json`ファイルが生成されます．

次に，HonKitをインストールします．以下のコマンドを実行してインストールします．

```bat
npm install honkit --save-dev
```

これでHonKitがプロジェクトのローカルにインストールされ，利用できるようになりました．

### Webページをビルドする
