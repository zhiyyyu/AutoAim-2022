import os
import sys

dir = './tmp_{}'

if __name__ == '__main__':

    for i in range(9):
        for file_name in os.listdir(dir.format(i)):
            
            file_path = os.path.join(dir.format(i), file_name)
            os.remove(file_path)
            print(file_path + ' has been removed.')
