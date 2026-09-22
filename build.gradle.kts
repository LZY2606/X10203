plugins {
    kotlin("jvm") version "2.1.0"
    application
}

repositories { mavenCentral() }

dependencies {
    implementation("io.ktor:ktor-server-core:3.1.2")
    implementation("io.ktor:ktor-server-netty:3.1.2")
    implementation("org.xerial:sqlite-jdbc:3.46.1.3")
    implementation("ch.qos.logback:logback-classic:1.5.12")
    testImplementation(kotlin("test"))
}

application {
    mainClass.set("atlas.MainKt")
}

tasks.test { useJUnitPlatform() }

kotlin { jvmToolchain(17) }
