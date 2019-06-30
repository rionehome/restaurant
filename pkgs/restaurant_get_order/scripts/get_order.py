#!/usr/bin/env python
# -*- coding: utf-8 -*-
import treetaggerwrapper as ttw
import os

TTWDIR = os.environ['HOME'] + '/tree-tagger-install/'
tagger = ttw.TreeTagger(TAGLANG='en', TAGDIR=TTWDIR)


def get_order_name(result):
    order_name = []
    for i in range(0, len(result)):
        if result[i][1].startswith('N'):
            order_name.append(result[i][0])
    return order_name


def main(txt):
    sentence = txt.decode('utf-8')  # treetagger用にutf-8へ
    result = []
    tags = tagger.TagText(sentence)
    
    for i in tags:
        li = i.split()
        result.append(li[:2])  # ('動詞', '品詞')
    
    print(get_order_name(result))
    return get_order_name(result)
