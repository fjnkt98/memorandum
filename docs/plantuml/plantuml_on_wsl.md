# WSLでPlantUMLを始める

## 環境

- Windows10
- WSL Ubuntu 20.04LTS
- Visual Studio Code

## PlantUMLとは

プレーンテキストでUMLの図を記述することができる言語のこと．

## 必要なソフトのインストール

PlantUMLの実行に必要なソフトウェアはすべて`apt`パッケージとして公開されているため，手軽にセットアップすることができます．

```bash
sudo apt update
sudo apt install default-jre graphviz plantuml
```

PlantUMLはJavaで実装されているため，Javaの実行環境が必要となります．
Javaの開発を行うのでなければ，Javaの実行環境`default-jre`をインストールするだけで十分でしょう．

## VSCode Extensionのインストール

VSCodeの[Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)を使って，WSLを開きます．

サイドバーから拡張機能を開き，「plantuml」と検索して一番上に出てくる[PlantUML](https://marketplace.visualstudio.com/items?itemName=jebbs.plantuml)拡張をインストールします．

これでWSL上でPlantUMLを利用する準備が整いました．
