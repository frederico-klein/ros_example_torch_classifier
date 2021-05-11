#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import numpy

### we copy paste what we did for the paper here, but need to read from a string topic and publish something

articles["titleabstract"] = articles["title"] + " " + articles["abstract"].fillna('')

articles["labelled"] = [not pd.isnull(s)
            and ("Included" in s or "Excluded" in s or "Maybe" in s) for s in articles['notes']]

print(articles.shape)

lab_df = articles.loc[articles['labelled']].copy()
print("Labelled data: {}".format(len(lab_df)))

# included list
includedlist = [("Included" in art) for art in articles["notes"].fillna('') ]

# we maybe do not want the maybes in the future?
maybelist = [("Maybe" in art) for art in articles["notes"].fillna('') ]

articles["class"] =  np.logical_or(includedlist, maybelist) ## python has a strange behaviour for or, but numpy saves the day
