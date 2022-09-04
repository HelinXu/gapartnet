'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-04 19:36:30
LastEditTime: 2022-09-05 02:56:17
Description: 
'''
import os
import time

ret = 99
resume_count = 0
while ret and resume_count < 10:
    resume_count += 1
    ret = os.system('python 05_render_check.py')
    time.sleep(1)
    ret = ret // 256
    print(f'resume: {resume_count}')
    
print('Finished')