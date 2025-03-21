# noqa: D104
from gymnasium.envs.registration import register

from engineering_sim.core import GoalEnv
from engineering_sim.envs.maze import maps

__version__ = "0.0.1"


def register_robotics_envs():
    """Register all environment ID's to Gymnasium."""

    def _merge(a, b):
        a.update(b)
        return a

    for reward_type in ["sparse", "dense"]:
        suffix = "Dense" if reward_type == "dense" else ""
        kwargs = {
            "reward_type": reward_type,
        }

        
        register(
            id=f"PointMaze_PATHPLANNING_MAP1{suffix}-v3",
            entry_point="engineering_sim.envs.maze.point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.PATH_PLANNING_FIG1,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_PATHPLANNING_MAP2{suffix}-v3",
            entry_point="engineering_sim.envs.maze.point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.PATH_PLANNING_FIG2,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_PATHPLANNING_MAP3{suffix}-v3",
            entry_point="engineering_sim.envs.maze.point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.PATH_PLANNING_FIG3,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        register(
            id=f"PointMaze_PATHPLANNING_MAP4{suffix}-v3",
            entry_point="engineering_sim.envs.maze.point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.PATH_PLANNING_FIG4,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_PATHPLANNING_MAP5{suffix}-v3",
            entry_point="engineering_sim.envs.maze.point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.PATH_PLANNING_FIG5,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_PATHPLANNING_MAP6{suffix}-v3",
            entry_point="engineering_sim.envs.maze.point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.PATH_PLANNING_FIG6,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_PATHPLANNING_MAP1{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_PATH_PLANNING_FIG1,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_PATHPLANNING_MAP2{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_PATH_PLANNING_FIG2,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_PATHPLANNING_MAP3{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_PATH_PLANNING_FIG3,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_PATHPLANNING_MAP4{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_PATH_PLANNING_FIG4,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_PATHPLANNING_MAP5{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_PATH_PLANNING_FIG5,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_PATHPLANNING_MAP6{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_PATH_PLANNING_FIG6,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        
        
        register(
            id=f"PointMaze_BARRIER_PATHPLANNING_MAP1{suffix}-v3",
            entry_point="engineering_sim.envs.maze.barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.BARRIER_PATH_PLANNING_FIG1,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_BARRIER_PATHPLANNING_MAP2{suffix}-v3",
            entry_point="engineering_sim.envs.maze.barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.BARRIER_PATH_PLANNING_FIG2,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_BARRIER_PATHPLANNING_MAP3{suffix}-v3",
            entry_point="engineering_sim.envs.maze.barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.BARRIER_PATH_PLANNING_FIG3,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_BARRIER_PATHPLANNING_MAP4{suffix}-v3",
            entry_point="engineering_sim.envs.maze.barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.BARRIER_PATH_PLANNING_FIG4,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_BARRIER_PATHPLANNING_MAP5{suffix}-v3",
            entry_point="engineering_sim.envs.maze.barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.BARRIER_PATH_PLANNING_FIG5,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_BARRIER_PATHPLANNING_MAP6{suffix}-v3",
            entry_point="engineering_sim.envs.maze.barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.BARRIER_PATH_PLANNING_FIG6,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        
        
        register(
            id=f"PointMaze_MA_BARRIER_PATHPLANNING_MAP1{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_BARRIER_PATH_PLANNING_FIG1,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_BARRIER_PATHPLANNING_MAP2{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_BARRIER_PATH_PLANNING_FIG2,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_BARRIER_PATHPLANNING_MAP3{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_BARRIER_PATH_PLANNING_FIG3,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_BARRIER_PATHPLANNING_MAP4{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_BARRIER_PATH_PLANNING_FIG4,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_BARRIER_PATHPLANNING_MAP5{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_BARRIER_PATH_PLANNING_FIG5,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"PointMaze_MA_BARRIER_PATHPLANNING_MAP6{suffix}-v3",
            entry_point="engineering_sim.envs.maze.multiagent_barrier_point_maze:PointMazeEnv",
            kwargs=_merge(
                {
                    "maze_map": maps.MA_BARRIER_PATH_PLANNING_FIG6,
                },
                kwargs,
            ),
            max_episode_steps=800,
        )
        
        register(
            id=f"RobotsArmEngSim{suffix}-v3",
            entry_point="engineering_sim.envs.fetch.robotsarm_engsim:RobotsArmEngSimEnv",
            kwargs=kwargs,
            max_episode_steps=50,
        )
        
        register(
            id=f"RobotsArmEngSim{suffix}-v4",
            entry_point="engineering_sim.envs.fetch.robotsarm_engsim_new:RobotsArmEngSimEnv",
            kwargs=kwargs,
            max_episode_steps=50,
        )
        
        register(
            id=f"MultiRobotsArmEngSim{suffix}-v3",
            entry_point="engineering_sim.envs.fetch.multi_robotsarm_engsim:MultiRobotsArmEngSimEnv",
            kwargs=kwargs,
            max_episode_steps=50,
        )
        
        register(
            id=f"MultiRobotsArmEngSim{suffix}-v4",
            entry_point="engineering_sim.envs.fetch.multi_robotsarm_engsim_new:MultiRobotsArmEngSimEnv",
            kwargs=kwargs,
            max_episode_steps=50,
        )


register_robotics_envs()


try:
    import sys

    from farama_notifications import notifications

    if (
        "engineering_sim" in notifications
        and __version__ in notifications["engineering_sim"]
    ):
        print(notifications["engineering_sim"][__version__], file=sys.stderr)
except Exception:  # nosec
    pass
