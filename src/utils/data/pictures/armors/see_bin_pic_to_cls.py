from cv2 import cv2
import shutil
import os
import sys

src_dir = './tmp_{}'
dst_dir = './{}'

if __name__ == '__main__':

    if len(sys.argv) < 2:
        index = input('请输入 tmp_{index}的index编号:')
    else :
        index = sys.argv[1]
    src_dir = src_dir.format(index)

    for file_name in os.listdir(src_dir):

        img = cv2.imread(os.path.join(src_dir, file_name))
        cv2.imshow("img", img)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.equalizeHist(img)
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        # _, img = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)

        cv2.imshow("final", img)

        key = cv2.waitKey(0)

        if key in [ord(str(i)) for i in range(9)]:
            
            shutil.move(os.path.join(src_dir, file_name), os.path.join(dst_dir.format(chr(key)), file_name))
            print('img has been moved to ./{}'.format(chr(key)))

        elif key == ord('q'):

            exit(0)
