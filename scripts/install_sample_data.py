#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'teach_spot'

    download_data(
        pkg_name=PKG,
        path='sample/data/walk_corridor_pr2.bag',
        url='https://drive.google.com/uc?id=1FAgj3pfRhWhQ3MF0xRcs7yunyzwu5ZHx',
        md5='e0db7051c72b77486b251858d5889b94',
        extract=False,
    )

if __name__ == '__main__':
    main()
    
