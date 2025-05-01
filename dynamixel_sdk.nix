{ lib, python311Packages, fetchFromGitHub }:

python311Packages.buildPythonPackage rec {
  pname = "dynamixel_sdk";
  version = "3.8.3";

  src = fetchFromGitHub {
    owner = "ROBOTIS-GIT";
    repo = "DynamixelSDK";
    rev = "3.8.3";
    sha256 = "18p45w9b01drgkfcssy674jqwj8s5811jzdhzz9yahw9zwssji4y";
  };

  sourceRoot = "source/python";

  doCheck = false;

  meta = with lib; {
    description = "Dynamixel SDK for Python from ROBOTIS";
    homepage = "https://github.com/ROBOTIS-GIT/DynamixelSDK";
    license = licenses.asl20;
  };
}

