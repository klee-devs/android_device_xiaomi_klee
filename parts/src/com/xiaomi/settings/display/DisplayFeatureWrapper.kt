/*
 * SPDX-FileCopyrightText: 2023-2025 Paranoid Android
 * SPDX-License-Identifier: Apache-2.0
 */

package com.xiaomi.settings.display

import android.os.Handler
import android.os.HandlerThread
import android.os.IBinder
import android.os.ServiceManager
import android.util.Log
import vendor.xiaomi.hardware.displayfeature_aidl.IDisplayFeature

object DisplayFeatureWrapper {
    private const val TAG = "DisplayFeatureWrapper"
    private val DEBUG = Log.isLoggable(TAG, Log.DEBUG)
    private val handlerThread = HandlerThread("DisplayFeature").apply { start() }
    private val handler = Handler(handlerThread.looper)

    data class Feature(val mode: Int, val value: Int, val cookie: Int)

    @Volatile private var displayFeature: IDisplayFeature? = null

    private val deathRecipient =
        IBinder.DeathRecipient {
            if (DEBUG) Log.d(TAG, "DisplayFeature service died")
            displayFeature = null
        }

    @Synchronized
    private fun getDisplayFeature(): IDisplayFeature? {
        displayFeature?.let {
            if (it.asBinder().isBinderAlive) return it
        }
        return try {
            val binder =
                ServiceManager.waitForService(
                    "vendor.xiaomi.hardware.displayfeature_aidl.IDisplayFeature/default",
                )
            val service = IDisplayFeature.Stub.asInterface(binder)
            service?.asBinder()?.linkToDeath(deathRecipient, 0)
            displayFeature = service
            if (DEBUG) Log.d(TAG, "Connected to DisplayFeature service")
            service
        } catch (e: Exception) {
            Log.e(TAG, "Failed to get DisplayFeature service", e)
            null
        }
    }

    fun setFeature(
        mode: Int,
        value: Int,
        cookie: Int,
    ) {
        setFeatures(Feature(mode, value, cookie))
    }

    fun setFeatures(vararg features: Feature) {
        handler.post {
            val service = getDisplayFeature()
            if (service == null) {
                if (DEBUG) Log.d(TAG, "DisplayFeature is null, skipping setFeatures")
                return@post
            }

            try {
                features.forEach { feature ->
                    if (DEBUG) {
                        Log.d(
                            TAG,
                            "setFeature: mode=" + feature.mode + " value=" + feature.value
                                + " cookie=" + feature.cookie,
                        )
                    }
                    service.setFeature(0, feature.mode, feature.value, feature.cookie)
                }
            } catch (e: Exception) {
                Log.e(TAG, "setFeatures failed!", e)
            }
        }
    }
}
