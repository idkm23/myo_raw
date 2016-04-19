'''
Created on Apr 8, 2016

@author: ymeng
'''

if __name__ == '__main__':
    import os
    from subprocess import call
    
    path = '../../data'
    work_path = os.path.join(path, 'work')
    
    os.chdir(work_path)
    call(['ls'])
    