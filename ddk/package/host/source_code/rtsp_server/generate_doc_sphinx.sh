#!/bin/bash
rm -rf rtsp_server_doc
sphinx-build -b html . rtsp_server_doc
sed -i 's/;max-width:800px;/;/' rtsp_server_doc/_static/css/theme.css
