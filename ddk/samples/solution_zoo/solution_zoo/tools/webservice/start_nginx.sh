#! /usr/bin/sh
start_nginx(){
  nginx_flag=$(ps | grep nginx | grep -v "grep")
  if [ -n "$nginx_flag" ]; then
    killall -9 nginx
  fi

  webservice_dir="webservice"
  if [ -z $HORIZON_APP_PATH ]; then
    echo "Have not set HORIZON_APP_PATH."
    if [ -d "/userdata/.horizon/bin/ai_express_webservice_display" ]; then
      echo "Has /userdata/.horizon/bin/ai_express_webservice_display, use it."
      webservice_dir="/userdata/.horizon/bin/ai_express_webservice_display"
    fi 
  elif [ -d "$HORIZON_APP_PATH/ai_express_webservice_display" ]; then
    echo "Have set HORIZON_APP_PATH, and have $HORIZON_APP_PATH/ai_express_webservice_display, use it."
    webservice_dir="$HORIZON_APP_PATH/ai_express_webservice_display"
  else
    echo "Already set HORIZON_APP_PATH, but has no webservice in it, nginx will not start!"
  fi
  cd $webservice_dir
  chmod +x ./sbin/nginx
  ./sbin/nginx -p .
  cd -
}
start_nginx