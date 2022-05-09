#!/bin/bash
rm -rf uvc_server_doc
sphinx-build -b html . uvc_server_doc
sed -i 's/;max-width:800px;/;/' uvc_server_doc/_static/css/theme.css
