export type SwerveModule = {
  speed: number,
  angle: number
}

export type SwerveModulesList = {
  front_left: SwerveModule,
  front_right: SwerveModule,
  rear_left: SwerveModule,
  rear_right: SwerveModule
}