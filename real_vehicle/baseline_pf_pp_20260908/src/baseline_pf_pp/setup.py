from setuptools import setup
setup(name='baseline_pf_pp', version='0.1.0', packages=['baseline_pf_pp'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/baseline_pf_pp']),
                  ('share/baseline_pf_pp', ['package.xml'])],
      install_requires=['setuptools'], license='MIT',
      entry_points={'console_scripts': [
          'safe_pf = baseline_pf_pp.safe_pf:main',
          'pure_pursuit = baseline_pf_pp.follower:main']})
