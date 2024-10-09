---
layout: post
title:  "ssh a nx orin and make its docker pull work"
date:   2024-10-09 21:23:00 +0800
tags: 
  - network
categories:
---

问题场景：使用个人PC通过 ssh 连接到 orin nx，但由于网络问题，orin的 `docker pull` 命令无法正常使用。而个人pc已开启代理，端口为127.0.0.1:8889（具体地址端口可在 v2ray 的首选项里查看）。

解决思路则是将 ssh 的 orin 的本地端口8889 关联到个人pc的 127.0.0.1:8889 上，之后在docker的代理配置文件里配置代理地址端口为 127.0.0.1:8889。

```sh
ssh -R 8889:127.0.0.1:8889 orin01@10.10.10.11
```

要在远程终端使用代理，则执行如下命令
```sh
export http_proxy="127.0.0.1:8889"
export https_proxy="127.0.0.1:8889"
export ftp_proxy="127.0.0.1:8889"
export ALL_PROXY="socks5://127.0.0.1:1089"
```

创建文件夹 `/etc/systemd/system/docker.service.d/`

创建文件 `/etc/systemd/system/docker.service.d/http-proxy.conf`，填写以下内容
```
[Service]
Environment="HTTP_PROXY=127.0.0.1:8889"
```

创建文件 `/etc/systemd/system/docker.service.d/https-proxy.conf`，填写以下内容
```
[Service]
Environment="HTTPS_PROXY=127.0.0.1:8889"
```

注意事项：
>Please check the dockerservice configuration /etc/systemd/system/docker.service.d/http-proxy.conf, if the configuration file is like this,
>```[Service]
>Environment="HTTP_PROXY=http://proxy.example.com:80"
>Environment="HTTPS_PROXY=https://proxy.example.com:443"
>```
>Please remove the https:// and http://

[stackoverflow - docker: Error response from daemon: Get https://registry-1.docker.io/v2/: proxyconnect tcp: EOF](https://stackoverflow.com/questions/64137423/docker-error-response-from-daemon-get-https-registry-1-docker-io-v2-proxyc)