#!/bin/bash
rm -rf model_inference_doc
sphinx-build -b html . model_inference_doc
sed -i 's/;max-width:800px;/;/' model_inference_doc/_static/css/theme.css
