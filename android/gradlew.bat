@echo off
setlocal
set DIRNAME=%~dp0
for %%i in ("%DIRNAME%.") do set APP_HOME=%%~fi
if defined JAVA_HOME (
  set "JAVA_EXE=%JAVA_HOME%\bin\java.exe"
) else (
  set "JAVA_EXE=java.exe"
)
"%JAVA_EXE%" -Dfile.encoding=UTF-8 -Xmx64m -Xms64m -Dorg.gradle.appname=gradlew -jar "%APP_HOME%\gradle\wrapper\gradle-wrapper.jar" %*
exit /b %ERRORLEVEL%
