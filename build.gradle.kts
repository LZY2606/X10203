plugins {
    kotlin("jvm") version "2.1.21"
    kotlin("plugin.serialization") version "2.1.21"
    application
}

group = "book"
version = "1.0.0"

repositories {
    mavenCentral()
}

dependencies {
    val ktorVersion = "3.2.2"
    implementation("io.ktor:ktor-server-core-jvm:$ktorVersion")
    implementation("io.ktor:ktor-server-netty-jvm:$ktorVersion")
    implementation("io.ktor:ktor-server-content-negotiation-jvm:$ktorVersion")
    implementation("io.ktor:ktor-serialization-kotlinx-json-jvm:$ktorVersion")
    implementation("ch.qos.logback:logback-classic:1.5.18")
    implementation("org.xerial:sqlite-jdbc:3.47.1.0")
    implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.8.0")

    testImplementation(kotlin("test"))
    testImplementation("io.ktor:ktor-server-test-host-jvm:$ktorVersion")
    testImplementation("org.junit.jupiter:junit-jupiter:5.11.4")
}

application {
    mainClass.set("book.app.MainKt")
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(17)
}
