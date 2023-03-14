import argparse
import logging
import os
import os.path as osp
import platform
import subprocess
from glob import glob
from distutils.dir_util import copy_tree
from distutils.file_util import copy_file
from pathlib import Path

import kkpyutil as util

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


def overwrite_copy(src: str, dst: str):
    if osp.isfile(src):
        copy_file(src, dst)

    if osp.isdir(src):
        copy_tree(src, dst)


def path_is_under(child: str, parent: str) -> bool:
    return Path(parent) in Path(child).parents


class Worker:
    def __init__(self, args):
        self.args = args
        self.wwiseRoot: str = os.getenv('WWISEROOT')
        self.wwiseSDKRoot: str = os.getenv('WWISESDK')
        self.wpScript = osp.join(self.wwiseRoot, 'Scripts/Build/Plugins/wp.py')
        self.ciDir = osp.dirname(__file__)
        self.projDir = osp.abspath(osp.join(self.ciDir, '..'))
        self.pluginName = osp.basename(self.projDir)
        self.repoDir = osp.abspath(osp.join(self.ciDir, '../../..'))
        self.terminatedWwise = False
        self.deployTargetConfigPath = osp.join(self.ciDir, '.deploy_target.json')
        # subclass fields
        self.deployTargetConfig = {}

    @staticmethod
    def get_platform_worker(args):
        system = platform.system()
        if system == 'Windows':
            return WindowsWorker(args)
        raise NotImplementedError(f'Not implemented for this platform: {system}')

    def main(self):
        self._load_deploy_targets()
        self._new_deploy_target()
        self._list_deploy_target()
        self._delete_deploy_target()
        util.save_json(self.deployTargetConfigPath, self.deployTargetConfig)

        if modified_target := (self.args.newDeployTarget or
                               self.args.listDeployTargets or
                               self.args.deleteDeployTarget):
            return

        self._validate_env()
        self._terminate_wwise()
        self._build()
        self._copy_authoring_plugin()
        self._apply_deploy_targets()
        self._reopen_wwise()

    def _load_deploy_targets(self):
        if osp.isfile(self.deployTargetConfigPath):
            self.deployTargetConfig = util.load_json(self.deployTargetConfigPath)

    def _new_deploy_target(self):
        if not self.args.newDeployTarget:
            return
        while True:
            name = input('Target name: ')
            if name not in self.deployTargetConfig.keys():
                break
            logging.warning(f'Target exists: {name}')

        engine_selections = {'ue', 'unity', 'other'}
        while True:
            engine = input('Game engine type(ue/unity/other): ')
            if engine in engine_selections:
                break
            logging.warning(f'Invalid engine: {engine}')

        while True:
            root = input('Game project root: ')
            if osp.isdir(root):
                break
            logging.warning(f'Directory not exists: {root}')

        self.deployTargetConfig[name] = {
            'engine': engine,
            'root': root
        }
        logging.info(f'New target created: {root}')

    def _list_deploy_target(self):
        if not self.args.listDeployTargets:
            return

        if not self.deployTargetConfig:
            logging.info('There is no deploy target.')

        for name, config in self.deployTargetConfig.items():
            print(name)
            engine = config["engine"]
            print(f'\t{engine=}')
            root = config["root"]
            print(f'\t{root=}\n')

    def _delete_deploy_target(self):
        if self.args.deleteDeployTarget in self.deployTargetConfig.keys():
            self.deployTargetConfig.pop(self.args.deleteDeployTarget)
            logging.info(f'Deleted target: {self.args.deleteDeployTarget}')

    def _validate_env(self):
        if self.wwiseRoot is None:
            raise EnvironmentError(f'Unknown env variable: WWISEROOT\n  - Try setting environment variables in Wwise '
                                   f'Launcher')

        if self.wwiseSDKRoot is None:
            raise EnvironmentError(f'Unknown env variable: WWISESDK\n  - Try setting environment variables in Wwise '
                                   f'Launcher')

        if not osp.isfile(self.wpScript):
            raise FileNotFoundError(f'"{self.wpScript}" not found.')

    def _build(self):
        logging.info('Building authoring plugin')
        util.run_cmd(
            [
                'python',
                self.wpScript,
                'build',
                'Authoring',
                '-c',
                self.args.configuration,
                '-x',
                'x64',
                '-t',
                'vc160'
            ],
            cwd=self.projDir
        )

    def _terminate_wwise(self):
        raise NotImplementedError('subclass it')

    def _copy_authoring_plugin(self):
        raise NotImplementedError('subclass it')

    def _apply_deploy_targets(self):
        raise NotImplementedError('subclass it')

    def _reopen_wwise(self):
        raise NotImplementedError('subclass it')


class WindowsWorker(Worker):
    def __init__(self, args):
        super().__init__(args)
        self.sharedPluginFiles = None

    def __lazy_query_shared_plugin_files(self) -> list:
        if self.sharedPluginFiles is not None:
            return self.sharedPluginFiles

        if self.args.configuration == 'Debug':
            # Use authoring plugin in debug mode for assert hook
            build_output_dir = osp.join(self.wwiseRoot, f'Authoring/x64/{self.args.configuration}/bin/Plugins')
        else:
            build_output_dir = osp.join(self.wwiseSDKRoot, f'x64_vc160/{self.args.configuration}/bin')

        self.sharedPluginFiles = list(filter(lambda x: not str(x).endswith('.xml'),
                                             glob(osp.join(build_output_dir, f'{self.pluginName}.*'))))
        return self.sharedPluginFiles

    def _build(self):
        super()._build()
        logging.info('Building shared plugin')
        util.run_cmd(
            [
                'python',
                self.wpScript,
                'build',
                'Windows_vc160',
                '-c',
                self.args.configuration,
                '-x',
                'x64'
            ],
            cwd=self.projDir
        )

    def _terminate_wwise(self):
        if self.args.forceCopyFile:
            cmd = ['taskkill', '/IM', 'wwise.exe', '/F']
            try:
                util.run_cmd(cmd)
                self.terminatedWwise = True
            except subprocess.CalledProcessError:
                pass

    def _copy_authoring_plugin(self):
        if self.args.configuration == 'Release':
            return

        build_output_dir = osp.join(self.wwiseRoot, f'Authoring/x64/{self.args.configuration}/bin/Plugins')
        src_files = glob(osp.join(build_output_dir, f'{self.pluginName}.*'))
        dst_dir = osp.join(self.wwiseRoot, f'Authoring/x64/Release/bin/Plugins')
        logging.info(f'Copy shared plugin "{src_files}", from "{build_output_dir}" to "{dst_dir}"')
        for src in src_files:
            dst = osp.join(dst_dir, osp.basename(src))
            overwrite_copy(src, dst)

    def _apply_deploy_targets(self):
        plugin_files = self.__lazy_query_shared_plugin_files()

        for name, config in self.deployTargetConfig.items():
            deploy_path = None
            if (engine := config['engine']) == 'ue':
                deploy_path = osp.join(f'{config["root"]}/Plugins/Wwise/ThirdParty/x64_vc160/Profile/bin')
            elif engine == 'unity':
                logging.warning('Unity is currently unsupported, skipped.')
            elif engine == 'other':
                deploy_path = osp.join(f'{config["root"]}')
            else:
                logging.warning(f'Unsupported engine type: {engine}, skipped.')

            if deploy_path is not None:
                logging.info(f'Apply deploy target: {name}')
                for output in plugin_files:
                    logging.info(
                        f'Copy shared plugin "{output}" to "{deploy_path}"')
                    dst = osp.join(deploy_path, osp.basename(output))
                    overwrite_copy(output, dst)

    def _reopen_wwise(self):
        if self.args.forceCopyFile and self.terminatedWwise:
            wwise_exe = osp.join(self.wwiseRoot, 'Authoring/x64/Release/bin/Wwise.exe')
            util.run_daemon([wwise_exe])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='Plugin dev ci build tool',
        add_help=True,
        epilog='Build both authoring and shared plugins, and copy the shared plugin to deploy target '
               'dir. Make sure wwise and game engine is not running.',
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument(
        '-c',
        '--configuration',
        action='store',
        choices=('Debug', 'Profile', 'Release'),
        dest='configuration',
        default='Debug',
        required=False,
        help='configuration to build (Debug, Release, Profile). Default value is Debug.'
    )
    parser.add_argument(
        '-f',
        '--force-copy-file',
        action='store_true',
        dest='forceCopyFile',
        required=False,
        default=False,
        help='Terminate Wwise process and copy plugin files, then reopen Wwise.'
    )
    parser.add_argument(
        '-n',
        '--new-deploy-target',
        action='store_true',
        dest='newDeployTarget',
        required=False,
        default=False,
        help='Create a new deploy target with interactive commandline.'
    )
    parser.add_argument(
        '-l',
        '--list-deploy-targets',
        action='store_true',
        dest='listDeployTargets',
        required=False,
        default=False,
        help='List all deploy targets.'
    )
    parser.add_argument(
        '-d',
        '--delete-deploy-target',
        action='store',
        dest='deleteDeployTarget',
        required=False,
        default='',
        help='Delete deploy targets by name.'
    )

    parsed_args = parser.parse_args()
    worker = Worker.get_platform_worker(parsed_args)
    worker.main()
