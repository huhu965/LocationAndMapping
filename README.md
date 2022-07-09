## git 使用
### git init 
初始化仓库
### git remote add origin https://[token]@github.com/huhu965/loam_frame
绑定远程仓库，用origin = 后面的url，这样提交用origin来代替url就可以了。  
目前github不再支持密码提交代码，都转为token了，绑定token的url时，就不用再输入账户和token了
### git remote set-url origin https://[token]@github.com/huhu965/loam_frame
token有时效限制，过期后要及时更新，用上面的指令来更新origin绑定的url。

## 说明
前端里程计借鉴LOAM的基础框架，融合了LEGO-loam针对分割聚类的优化，IMU预积分提供预测值，帧到地图匹配提高精度。
