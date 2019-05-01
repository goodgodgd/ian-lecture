---
 layout: post
title:  "[Python] Implement GUI Text Editor"
date:   2019-04-30 09:00:01
categories: jekyll
---



# GUI Text Editor

지금까지는 `QPushButton`과 `QLabel`만 써서 PyQt의 GUI를 사용하는 흐름을 배웠다. 이제부터는 GUI 텍스트 에디터를 구현하는 실습을 하면서 다양한 GUI 객체들을 활용하는 법을 배울것이다. 여기서는 그 중에서 자주 쓰이는 GUI 객체들만 활용하고 알아보고 나머지는 [Qt Documentation](<https://doc.qt.io/qt-5/index.html>)에서 검색해서 용법을 찾아보면 된다.  

새로운 프로젝트를 시작하는 것이므로 파이참에서 새로운 `text_editor`란 이름의 새로운 프로젝트를 만들고 시작하자.  




## 1. QTextEdit

`QTextEdit`은 입출력 가능한 텍스트 창이다. 프로그램에서 텍스트를 출력할 수도 있고 사용자가 텍스트를 입력할 수도 있다. 일단 MainWindow에 `QTextEdit`을 추가해 화면에 띄워보자. 객체의 이름은 `textEdit` 그대로 두고 프로젝트 폴더에 `text_editor.ui`로 저장하자.

![QTextEditor](/ian-lecture/assets/text_editor/textedit.png)



이후 UI 파일을 불러서 GUI를 화면에 띄우는 간단한 코드를 작성한다.

```python
import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("text_editor.ui", self)

def main():
    app = QApplication(sys.argv)
    editor = MyWindow()
    editor.show()
    app.exec_()

if __name__ == "__main__":
    main()
```



## 2. QMenu, QAction

이제 화면 상단에 파일을 열고 닫을 수 있는 메뉴를 만들고 기능을 구현한다. QtDesigner에서 윈도우 상단의 `TypeHere`에 `File`이란 메뉴(QMenu)를 추가하고 그 아래 `Open`과 `Save`라는 액션(QAction)을 추가한다. 그러면 자연스럽게 Object Inspector 메뉴에 아래와 같이 객체들이 추가된다.

![QTextEditor](/ian-lecture/assets/text_editor/menu.png)

![QTextEditor](/ian-lecture/assets/text_editor/menu_obj.png)

이제 코드에서 action을 클릭했을 때 실행할 Slot 함수를 연결해주자.

><Signal 사전> QAction
>
>**triggered**: 상단 메뉴의 action을 클릭했을 때 발생하는 Signal이다.

`actionOpen.triggered`는 `open_file` 함수와 연결했고 `actionSave.triggered`는 `save_file` 함수와 연결하였다. 

```python
class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("text_editor.ui", self)
        self.setup_ui()

    def setup_ui(self):
        self.actionOpen.triggered.connect(self.open_file)
        self.actionSave.triggered.connect(self.save_file)
```



`open_file` 함수에서는 `QFileDialog.getOpenFileName()`를 써서 다이얼로그에서 파일을 선택하여 파일명을 받을 수 있게 했다. 이후 파일 내용을 읽어 `textEdit`객체에 쓰기 위해 `setText()` 함수를 썼다.

```python
    def open_file(self):
        filename = QFileDialog.getOpenFileName(filter="Text files (*.txt)")
        filename = filename[0]
        print("open file:", filename)
        if not filename:
            return
        with open(filename, "r") as f:
            text = f.read(10000)
            self.textEdit.setText(text)
```



`close_file` 함수에서는 `QFileDialog.getSaveFileName()` 함수를 써서 다이얼로그에서 저장할 파일명을 지정하였다. 보통 텍스트 객체에서 현재 표시하고 있는 텍스트를 읽어오는 함수는 `text()`이지만 `QTextEdit`에서는 `toPlainText()`를 통해 현재 표시된 텍스트를 읽을 수 있다.

```python
    def save_file(self):
        filename = QFileDialog.getSaveFileName(filter="Text files (*.txt)")
        filename = filename[0]
        print("open file:", filename)
        if not filename:
            return
        with open(filename[0], "w") as f:
            f.write(self.textEdit.toPlainText())
```



## 3. QStatusBar

텍스트를 읽어나 수정할 때 글자 수와 현재 위치를 윈도우 아래 statusbar에 출력해보자. 새 UI를 만들때 MainWindow를 선택하면 기본으로 윈도우 아래 `statusbar` 객체가 포함돼있다. 먼저 `textEdit`에서 커서가 이동할 때 발생하는 Signal과 이를 처리하는 Slot 함수(update_status)를 연결해야 한다.

> <Signal 사전> QTextEdit
>
> **cursorPositionChanged**: QTextEdit 객체에서 커서 위치가 변할 때 발생
>
> **textChanged**: QTextEdit 객체에서 텍스트가 변할 때 발생

```python
    def setup_ui(self):
        self.actionOpen.triggered.connect(self.open_file)
        self.actionSave.triggered.connect(self.save_file)
        self.textEdit.cursorPositionChanged.connect( self.update_status)
```

`update_status()` 함수에서는 현재 커서 정보를 받기 위해 `textEdit.textCursor()` 함수를 통해 `QTextCursor` 객체를 받아와 커서 정보를 얻었다. `position()`은 현재 커서 위치고 `anchor()`는 drag를 시작했을 때의 커서 위치라서 평소엔 두 함수가 같은 값을 리턴하고 drag를 하면 다른 값을 리턴하게 된다.

```python
    def update_status(self):
        text_len = len(self.textEdit.toPlainText())
        cursor_pos = self.textEdit.textCursor().position()
        cursor_anc = self.textEdit.textCursor().anchor()
        if cursor_pos == cursor_anc:
            status = f"text length:{text_len}, cursor position: {cursor_pos}"
        else:
            status = f"text length:{text_len}, cursor range: {cursor_anc}~{cursor_pos}"
        self.statusbar.showMessage(status)
```



## 4. QComboBox

`QComboBox`는 여러개의 목록에서 하나를 선택할 수 있는 GUI다. 이를 이용해 새로 입력될 텍스트의 폰트(font)를 선택할 수 있도록 할 것이다. 편리하게도 QtDesigner에서는 이미 폰트 선택만을 위한 `QFontComboBox`라는 클래스를 제공하고 있고 이것만 추가하면 시스템의 모든 폰트를 자동으로 불러올 수 있으나 여기서는 학습 목적을 위해 기본적인 `QComboBox`를 쓰도록 한다.  

QtDesigner에서 왼쪽 Input Widgets 중에서 Combo Box를 선택하여 `textEdit` 옆에 놓는다. 객체 이름은 기본 값인 `comboBox` 그대로 둔다. UI 파일을 저장하고 코드를 수정한다.  폰트를 선택하고 이를 `textEdit`에서 반영하기 위해서는 다음 단계가 필요하다.

1. `comboBox` 객체에 폰트 목록을 추가하기
2. 시작할 때 기본 선택된 폰트를 `textEdit`에 반영하기
3. `comboBox`에서 폰트가 바뀔 때 자동으로 `textEdit`에 반영하기

이를 초기 설정을 하는 `setup_ui()` 함수에 반영하고 Slot 함수인 `change_font()` 함수를 만들었다.

```python
    def setup_ui(self):
        self.actionOpen.triggered.connect(self.open_file)
        self.actionSave.triggered.connect(self.save_file)
        self.textEdit.cursorPositionChanged.connect( self.update_status)
        self.comboBox.addItems(["굴림", "돋움", "바탕"])
        self.textEdit.setFontFamily(self.comboBox.currentText())
        self.comboBox.currentIndexChanged.connect( self.change_font)

    def change_font(self, cur_index):
        print("comboBox index:", cur_index)
        self.textEdit.setFontFamily(self.comboBox.currentText())
```



1. `comboBox.addItems()`로 폰트 목록을 추가
2. `comboBox.currentText()`로 현재 선택된 폰트를 읽어서 `textEdit.setFontFamily()` 함수로 폰트를 변경
3. `comboBox.currentIndexChanged`라는 Signal을 `change_font()`함수와 연결

> <Signal 사전> QComboBox
>
> **currentIndexChanged**: combo box에서 선택된 item이 바뀔 때 발생, slot 함수에서 index를 입력인자로 받을 수 있다.
>
> **currentTextChanged**: 사용자가 다른 item을 선택하거나 코드에서 item의 텍스트를 수정할 때 발생, slot 함수에서 선택된 item의 문자열을 입력인자로 받을 수 있다.

이를 실행하여 폰트별로 글자를 쓴 결과는 다음과 같다.

![comboBox](/ian-lecture/assets/text_editor/combobox.png)



## 5. QGroupBox, QRadioButton

Radio button은 여러 개의 버튼 중에 하나만 선택할 수 있는 버튼이다. 여기서는 이를 이용해 새로 입력될 글자의 색깔을 결정하고자 한다.









