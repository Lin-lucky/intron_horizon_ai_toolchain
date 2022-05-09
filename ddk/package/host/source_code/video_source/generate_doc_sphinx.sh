#!/bin/bash
rm -rf video_source_doc
sphinx-build -b html . video_source_doc
sed -i 's/;max-width:800px;/;/' video_source_doc/_static/css/theme.css
