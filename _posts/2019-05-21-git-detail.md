---
layout: post
title:  "[Git] Git into the Git"
date:   2019-05-21 09:00:01
categories: 2019-1-micro
---



# 0. Quick Review

다음은 상황 별로 Git을 활용한 버전 관리를 하는 흐름을 요약해서 보여준다. 옵션 중 `[]`로 표시된 것은 선택 옵션이고 `<>`로 표시한 것은 필수 옵션이다.

## A. 저장소를 처음 만들 때

1. GitHub에서 새로운 저장소를 만든다.

2. 로컬에서 디렉토리를 만들고 저장소를 초기화 한다.

   `git init`

3. 사용자 설정: 이 로컬 저장소를 수정하는 사람의 정보 등록 -> commit에 기록됨

   `git config [--global] user.name <사용자 이름>`

   `git config [--global] user.email <email 주소>`

4. 원격 저장소 지정하기

   `git remote add <저장소 별명 보통은 origin> <저장소 주소>`

5. 새 파일을 작성하거나 기존 파일을 저장소에 붙여놓고 파일을 **stage**하기 (버전 관리되는 파일로 등록하기)

   `git add <파일>` : 특정 파일만 stage하기

   `git add .` : 현재 폴더 아래 있는 모든 수정 사항 stage 하기

6. 언제나 현재 상태를 확인하고 싶을 땐

   `git status`

7. Stage한 저장소 전체의 상태를 복원 가능한 *버전*으로 메시지와 함께 저장하기

   `git commit -m <변경 사항 메시지>`

8. 파일 변경/추가 - add(4) - commit(6) 반복 하기

9. 원격 저장소에 그 동안 쌓은 commit들 업로드하기

   `git push origin master`



## B. 새로운 환경에서 원격 저장소 받아서 작업하기

1. 원격 저장소를 복사한 로컬 저장소를 만든다. 디렉토리 이름을 지정하지 않으면 저장소 이름과 같은 이름의 디렉토리가 생긴다.

   `git clone <저장소 주소> [로컬 저장소 디렉토리 이름]`

   Git이 관리하는 디렉토리로 들어가야 Git 명령어로 버전을 관리할 수 있다.

   `cd <로컬 저장소 디렉토리 이름>`

2. 사용자 설정: 이 로컬 저장소를 수정하는 사람의 정보 등록 -> commit에 기록됨

   `git config [--global] user.name <사용자 이름>`

   `git config [--global] user.email <email 주소>`

3. Clone을 한 경우 원격 저장소 주소는 자동으로 `origin`이란 별명으로 등록되어 있다. 로컬 저장소에 연결된 원격 저장소를 확인하는 방법은

   `git remote -v`

4. 저장소 내의 파일 수정/추가 후 변경 사항 stage하기

   `git add .`

5. Stage한 저장소 전체의 상태를 복원 가능한 *버전*으로 메시지와 함께 저장하기

   `git commit -m <변경 사항 메시지>`

6. 파일 변경/추가 - add(4) - commit(6) 반복 하기

7. 원격 저장소에 그 동안 쌓은 commit들 업로드하기

   `git push origin master`



## C. 기존 로컬 저장 저장소에서 다시 작업하기

1. A 방법이든 B 방법이든 기존에 작업하던 로컬 저장소가 있는데 (자신이든 다른 사람이든) 다른 로컬 저장소에서 push를 해서 현재 로컬 저장소에 없는 commit이 원격 저장소에 있다면 이를 먼저 받고 작업을 재개해야 한다.

   `git pull`

2. 이후 작업을 하면서 B의 4~7을 반복하면 된다.



# 1. Git Verb 정리

## 1. 버전을 쌓는 Verb



### git init

>  현재 디렉토리를 Git이 관리하는 저장소로 초기화 한다.

---



### git remote

> 로컬 저장소와 연결된 원격 저장소들을 관리한다.
>
> `git remote -v` : 현재 가지고 있는 원격 저장소 목록 보여주기
>
> `git remote add <name>  <url>` : 원격 저장소 주소 url을 name이란 별명으로 등록
>
> `git remote rename <old_name>  <new_name>` : 원격 저장소 별명을 "old_name"에서 "new_name"으로 변경
>
> `git remote remove <name>` : 원격 저장소 삭제
>
> `git remote set-url <name> <new_url>`: name 원격 저장소의 주소를 new_url로 수정

---



### git config

> Git에 대한 설정을 변경할 수 있는 명령어. 설정할 수 있는 변수가 수백가지지만 대부분은 사용자 등록 정도만 사용한다.
>
> `git config [--global] user.name <name>` : 사용자 이름을 등록한다. `--global` 옵션을 쓰면 이 PC의 모든 로컬 저장소에 기본 사용자가 된다. `--global`을 빼면 현재 저장소에만 적용이 된다.
>
> `git config [--global] user.email <email>`: 사용자의 이메일을 등록한다. `--global`의 용도는 위와 같다.
>
> `git config [--global] --list` : Git의 모든 설정 정보를 조회한다. `--global`을 쓰면 `--global` 옵션을 주고 설정한 전역 설정 정보를 조회한다.

---



### git status

> 최신 commit으로부터 변경 사항이 있는 파일들을 상태별로 묶어서 보여준다. Untracked, Not staged, Staged, Changes to be commited 등의 상태가 있다. 그리고 현재 상태에서 쓸만한 명령어 추천해준다.

---



### git add

> Untracked 상태의 파일을 인덱스에 추가하거나 변경 사항을 stage 한다.
>
> `git add <filename>` : 특정 파일을 stage 한다.
>
> `git add <pattern like *.txt>` : 현재 디렉토리에서 패턴과 일치하는 모든 파일을 stage 한다.
>
> `git add .` : 현재 디렉토리와 하위 디렉토리의 모든 변경된 파일들을 stage 한다. 하지만 파일을 삭제한 사실을 stage 해주진 않는다.
>
> `git add -A`: 현재 디렉토리와 하위 디렉토리의 모든 변경된 파일들과 삭제된 파일들을 stage 한다.

---



### git commit

> Staged 변경 사항까지 들어간 저장소의 스냅샷을 저장한다. commit을 하면 Stage 한 상태까지를 저장하고 hash 혹은 checksum을 부여한다. (checksum과 hash는 비슷하게 쓰이지만 방식이 다르고 git에서는 checksum이 더 정확한 용어지만 hash라고 많이 부른다.)  Hash는 코드로부터 자동으로 생성되는 40자리 문자로서 commit의 ID 같은 역할을 한다. Hash는 `git log` 명령어를 통해 확인할 수 있다. 이 hash를 이용해 나중에 언제든 예전에 commit한 상태로 돌아갈 수 있다.
>
> `git commit -m <message>` : 현재 stage된 변경사항을 message, author, hash와 함께 저장한다.  `-m` 옵션이 없으면 Git 기본 에디터가 실행되서 그곳에서 메시지를 작성하게 한다. 그냥 커맨드에서 -m 옵션을 써서 메시지를 입력하는게 낫다.
>
> `git commit --amend -m <message>` : 직전 commit에서 빠진게 있을 때 변경 사항을 추가하고 add 한 뒤 `--amend` 옵션을 붙여 commit하면 직전 commit을 없애고 추가 변경 사항까지 합친 새로운 commit을 만든다. 변경 사항이 없더라도 단순히 commit message를 다시 쓰고자 할 때도 사용된다.

---



### git clone

> 원격 저장소를 복사한 로컬 저장소를 만든다. 저장소를 복사한다는 의미는 모든 브랜치의 모든 commit을 다운로드 받는다는 것이다. GitHub에서 "Download Zip"으로도 코드를 받을 수 있지만 과거 이력없이 최신 상태의 코드만 받게 된다.
>
> `git clone <repository_url> [dir_name]` : `repository_url` 주소의 원격 저장소를 복사한다. 기본적으로는 저장소 이름과 같은 디렉토리가 생기고 뒤에 `dir_name`을 지정하면 그 이름으로 디렉토리가 생긴다.
>
> `git clone -o <name> <repository_url>` : -o 옵션을 통해 원격 저장소의 이름을 `origin`이 아닌 다른 이름으로 지정할 수 있다.

---



### git pull

> 원격 저장소에서 commit을 받아와 commit의 변경 사항을 로컬 저장소의 코드에 합친다.(merge) Pull을 실행하기 전에 반드시 로컬 저장소의 상태는 모든 것이 commit이 된 "Unmodified" 상태여야 pull을 할 수 있다. Pull은 사실 모든 commit을 내려받는 `git fetch`와 내려받은 commit들과 현재 로컬 파일에 반영하는 (합치는) `git merge FETCH_HEAD` 두 명령어를 결합한 것이다. 따라서 pull에는 merge와 관련된 옵션들이 있다.
>
> `git pull` : 원격 저장소의 모든 브랜치의 commit들을 로컬 저장소에 받고 각 브랜치를 모두 merge 한다. 원격의 master는 로컬의 master와 합치고 원격의 some_branch는 로컬의 some_branch와 합친다.
>
> `git pull <remote> <local_branch>` : 특정 브랜치(local_branch)만 변경 사항(commit)을 내려받고 합친다.
>
> `git pull [--ff / --no-ff / --only-ff]` : merge 방식에 fast-forward 방식과 non-fast-forward 방식이 있는데 두 방식에 대한 설명은 [이곳](<https://backlog.com/git-tutorial/kr/stepup/stepup1_4.html>)에서 확인할 수 있다. `--only-ff`는 fast-forward 방식이 가능할 때만 merge를 하라는 것이다.

---



### git push

> 로컬 저장소의 commit을 원격 저장소로 올린다. 현재 로컬 저장소의 파일 상태나 Stage 여부에 상관없이 오직 commit에 들어간 변경 사항만 원격 저장소로 올린다.
>
> `git push origin master` : Git 초보자들이 가장 많이 쓰는 명령어 중 하나이다. 저장소를 clone 받으면 `master`라는 기본 브랜치가 선택되고 원격 저장소는 자동으로 `origin`이란 이름으로 저장된다. 그래서 `master` 브랜치에서 작업 후 commit을 업로드 할 때 이 명령어를 쓰게된다.
>
> `git push <remote_repository> <local_branch>` : local_branch의 commit들을 원격 저장소의 같은 이름의 브랜치에 올린다. 예를 들어 `git push origin master`는 로컬 저장소의 `master` 브랜치의 commit들을 원격 저장소의 `master` 브랜치(origin/master)에 올린다는 것이다. 원격 저장소에 local_branch가 없을 경우 GitHub에서 자동으로 같은 이름의 브랜치를 만들어준다.
>
> `git push <remote_repository> <local_branch>:<remote_branch>` : local_branch의 commit들을 원격 저장소의 remote_branch에 반영한다.
>
> `git push --all` : 모든 로컬 브랜치를 한번에 push 한다.
>
> `git push --tags` :  기본적인 push는 tag를 자동으로 push 하지않는다. --tag 옵션으로 모든 tag를 push 할 수 있다. Tag에 대해서는 추후 배운다.



## 2. 버전을 관리하는 Verb



### git rm

> rm은 remove의 약자로 파일을 삭제하고 삭제한 상태를 stage한다. 즉 파일을 삭제한 후 `git add -A` 한 것과 같다.  Git으로 버전 관리되는 파일은 가급적 rm을 이용해 삭제하는 것이 좋다.
>
> `git rm <file_name>` : 지정한 파일을 삭제하고 stage한다.
>
> `git rm <file_pattern>` : 패턴과 일치하는 모든 파일을 삭제하고 stage한다. 예를들어 `git rm *.txt` 라고 하면 모든 텍스트 파일을 삭제하는 것이다.
>
> `git rm -rf <dir_name>` : 디렉토리를 삭제할 때는 -rf 옵션을 줘야한다.
>
> `git rm --cached <file_name>` : 실제 파일은 삭제하지 않고 파일을 인덱스에서 제외하여 Untracked 상태로 만든다.

---



### git mv

> mv는 move의 약자로 파일을 이동하고 이동한 상태를 stage한다. 버전 관리되는 파일을 A 디렉토리에서 B 디렉토리로 옮겨버리면 A에서는 파일이 삭제되고 B에는 새 파일이 추가된 것으로 인식한다. 이를 stage하기 위해서는 역시 `git add -A`를 해줘야한다. 그러므로 파일을 이동할 때는 가급적 mv를 이용하는 것이 좋다. 
>
> mv는 또한 단순히 파일의 이름을 바꾸는데도 사용된다. 같은 경로에 다른 이름으로 옮기면 파일명 변경이 된다.
>
> `git mv <src_file> <dst_file>` : src_file을 dst_file로 이름을 바꾼다.
>
> `git mv <src_file> <dst_dir>` : src_file을 dst_dir 디렉토리로 옮긴다.
>
> `git mv <src_file> <dst_dir/dst_file>` : src_file을 dst_dir 디렉토리 아래 dst_file이란 이름으로 옮긴다.

------



### git log

> 현재까지 쌓인 commit들을 볼 수 있다. Commit의 hash code, 저자 정보, 날짜 시간, commit 메시지를 보여준다. log 보기 상태에서 'q'를 눌러야 빠져나올 수 있다.
>
> `git log` : 기본
>
> `git log --oneline` : 한 줄에 commit 하나씩 보여주기
>
> `git log --graph` : 변경 이력을 그래프로 보여주기
>
> `git log -p` : 각 commit의 텍스트 변경 사항을 직접 보여주기
>
> `git log -n` : 최근 n개의 commit만 보기
>
> `git log  --since="4 weeks"` : 지난 4주 동안의 로그 보기. 시간 단위는 seconds/minutes/hours/days/weeks/months/years 등 모두 가능
>
> `git log --before="2 days"` : 2일 전까지의 로그 보기

------



### git reset

> 버전 관리에서 매우 중요하면서도 조심히 써야 하는 verb다. 과거 버전으로 돌아가는 명령어다. 돌아갈 버전은 특정 commit으로 지정하는데 commit의 hash code나 가장 최근 commit을 나타내는 HEAD로 commit을 지정한다. 세 가지 방식이 있는데 유의해서 써야 한다. 자세한 설명은 [이곳]([https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-Reset-%EB%AA%85%ED%99%95%ED%9E%88-%EC%95%8C%EA%B3%A0-%EA%B0%80%EA%B8%B0](https://git-scm.com/book/ko/v2/Git-도구-Reset-명확히-알고-가기))에서 그림을 보는것이 좋다.
>
> `git reset --soft <past_commit> [filename]` : 작업 트리의 내용은 바꾸지 않고 HEAD만 "past_commit" 으로 변경한다. "past_commit" 이후의 변경 사항은 stage 되어있다. filename을 쓰면 특정 파일만 변경하고 
>
> `git reset --mixed <past_commit> [filename]` :  --mixed는 기본 옵션이다. 작업 트리의 내용은 바꾸지 않고 HEAD를 "past_commit" 으로 변경한다. "past_commit" 이후의 변경된 파일은 Unstaged 상태가 된다.
>
> `git reset --hard <past_commit>` : 작업 트리의 내용을 "past_commit" 버전으로 바꾸고 이후의 commit들은 삭제해버린다.
>
> `git reset <filename>` : 특정 파일에서 stage한 내용을 취소하여 Unstaged (Modified or Untracked) 상태로 만든다. add 명령을 취소한다고 보면된다.

------



### git merge

> sdf

------



### git diff

> sdf

------



### git branch

> sdf

------



### git checkout

> sdf

------





# 2. Git을 이용한 프로젝트 관리

코드비전 프로젝트 만들어서 관리 예시로 설명

- gitignore
- 



[https://medium.com/@joongwon/git-git-%EB%AA%85%EB%A0%B9%EC%96%B4-%EC%A0%95%EB%A6%AC-c25b421ecdbd](https://medium.com/@joongwon/git-git-명령어-정리-c25b421ecdbd)

<https://blog.outsider.ne.kr/572>

<https://brainbackdoor.tistory.com/15>

