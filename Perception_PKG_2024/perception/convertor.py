import os

def rename_files_and_folders(old_names, new_names):
    for old_name, new_name in zip(old_names, new_names):
        if os.path.exists(old_name):
            os.rename(old_name, new_name)
        else:
            print(f"{old_name} does not exist.")
            return False
    return True

def check_versions(path):
    # Paths to be checked
    v7 = [os.path.join(path, 'utils_v7'), os.path.join(path, 'setup_v7.py'), os.path.join(path, 'models_v7')]
    v9 = [os.path.join(path, 'utils_v9'), os.path.join(path, 'setup_v9.py'), os.path.join(path, 'models_v9')]
    
    # Check v7 paths
    v7_exists = all(os.path.exists(p) for p in v7)
    
    # Check v9 paths
    v9_exists = all(os.path.exists(p) for p in v9)
    
    if v7_exists:
        return 9
    elif v9_exists:
        return 7
    else:
        return None

def c7to9(path):
    # Paths to be changed
    original = [os.path.join(path, 'utils'), os.path.join(path, 'setup.py'), os.path.join(path, 'models')]
    v7 = [os.path.join(path, 'utils_v7'), os.path.join(path, 'setup_v7.py'), os.path.join(path, 'models_v7')]
    v9 = [os.path.join(path, 'utils_v9'), os.path.join(path, 'setup_v9.py'), os.path.join(path, 'models_v9')]

    # Check if v9 files/folders exist
    if all(os.path.exists(p) for p in v9):
        # Rename original to v7
        if rename_files_and_folders(original, v7):
            # Rename v9 to original
            if not rename_files_and_folders(v9, original):
                # Rollback if renaming fails
                rename_files_and_folders(v7, original)
    else:
        print("Required v9 files/folders do not exist.")

def c9to7(path):
    # Paths to be changed
    original = [os.path.join(path, 'utils'), os.path.join(path, 'setup.py'), os.path.join(path, 'models')]
    v7 = [os.path.join(path, 'utils_v7'), os.path.join(path, 'setup_v7.py'), os.path.join(path, 'models_v7')]
    v9 = [os.path.join(path, 'utils_v9'), os.path.join(path, 'setup_v9.py'), os.path.join(path, 'models_v9')]
    print(os.path.join(path, 'utils'))
    # Check if v7 files/folders exist
    if all(os.path.exists(p) for p in v7):
        # Rename original to v9
        if rename_files_and_folders(original, v9):
            # Rename v7 to original
            if not rename_files_and_folders(v7, original):
                # Rollback if renaming fails
                rename_files_and_folders(v9, original)
    else:
        print("Required v7 files/folders do not exist.")

path="/ros2_ws/src/perception_ws/"

path_dict = {
    "deli": "delivery_yolo",
    "d": "delivery_yolo",
    "traffic": "traffic_yolo",
    "t": "traffic_yolo",
    "amz": "amz_yolo",
    "a": "amz_yolo"
}

print("현재 모든 버전: ")
for i in ("deli","traffic","amz"):
    print(path_dict[i], ":", check_versions(path+path_dict[i]))

breakp=False

cvt=[0,0,0,0,0,0,0,c9to7,0,c7to9]
while(1):
    if(breakp):
        break


    while 1:
        package = input("버전을 수정할 패키지를 입력하시오..\ndeli, traffic, amz, all,\nd,  t,  a,  al   ::  ")
        if not (package in ("deli","traffic","amz","all","d","t","a","al   ")):
            print(f"{package}는 올바른 패키지명이 아닙니다.")
        else:
            if package=="all" or package=="al":
                for i in ("deli","traffic","amz"):
                    v=input("수정할 버전 명을 입력하시오(현재 버전이 7이면 9를 입력.) :\n뒤로가기는 b   ::  ")
                    if v=="7" or v=="9":
                        v=int(v)
                        for i in ("deli","traffic","amz"):
                            if v!=check_versions(path+path_dict[i]):
                                cvt[v](path+path_dict[i])
                    elif v=="b":
                        break
                    else:
                        print("잘못 입력하셨습니다. 입력 가능 옵션 : '7' 혹은 '9' 혹은 'b' .")
            else:
                print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                print("수정할 패키지:", path_dict[package])

                print("현재 버전 : ",check_versions(path+path_dict[package]))
                v=input("수정할 버전 명을 입력하시오(현재 버전이 7이면 9를 입력.) :\n뒤로가기는 b   ::  ")
                if v=="7":
                    c9to7(path+path_dict[package])
                    breakp=True
                    break
                elif v=="9":
                    c7to9(path+path_dict[package])
                    breakp=True
                    break
                elif v=="b":
                    break
                else:
                    print("잘못 입력하셨습니다. 입력 가능 옵션 : '7' 혹은 '9' 혹은 'b' .")
print("경로 및 파일 명 수정이 완료되었습니다. 현재 버전 : ")
for i in ("deli","traffic","amz"):
    print(path_dict[i], ":", check_versions(path+path_dict[i]))
print("     **object_detection 내부에서 사용하는 학습파일 버전을 확인해주세요!")
